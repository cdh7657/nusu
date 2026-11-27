/*
  누수알림기 알림 서버 (Firebase Cloud Functions, 2세대)
  - leakAlert  : 장치가 /leak/{코드}/alerts 에 알림을 쓰면 → 고객 폰(/leak/{코드}/tokens)과 관리자 폰(/leakAdmin/tokens)으로 푸시
                 학습 기간(설치 후 1달) 중 미세누수 알림은 관리자에게만 보냄
  - leakOffline: 30분마다 장치 연결을 확인 → 90분 넘게 소식이 없으면 한 번 알림
  - leakAdmin  : 관리 화면(leak-admin.html)용 창구. 고객 목록은 DB에서 아무나 못 읽게 막고, 관리 키가 맞을 때만 이걸로 보여 줌
  배포: leak/설치안내.md 의 "알림 서버 올리기" 참고
*/
const { onValueCreated } = require("firebase-functions/v2/database");
const { onSchedule } = require("firebase-functions/v2/scheduler");
const { onRequest } = require("firebase-functions/v2/https");
const admin = require("firebase-admin");

admin.initializeApp();
const REGION = "asia-southeast1";
const INSTANCE = "daewon-booking-default-rtdb";
const APP = "https://cdh7657.github.io/nusu/";
const ICON = APP + "leak/icon.png";

function words(a, owner) {
  const house = owner.house || owner.name || "우리 집";
  const learn = a.learning ? "[학습 중·관리자만] " : "";
  if (a.type === "flow") return {
    title: learn + "🚨 물이 1시간 넘게 계속 흐르고 있어요",
    body: `${house} · 물을 쓰는 중이 아니라면 계량기 밸브를 잠그고 눌러 주세요.`,
  };
  if (a.type === "offline") return {
    title: "📡 누수알림기 연결이 끊겼어요",
    body: `${house} · 전원이나 와이파이를 확인해 주세요. 정전이면 전기가 들어오면 다시 연결돼요.`,
  };
  const per = a.period ? ` (${Math.round(a.period)}초마다 채움)` : "";
  return {
    title: learn + "💧 물을 안 쓰는데 조금씩 새는 것 같아요",
    body: `${house} · 감압밸브가 혼자 물을 채우고 있어요${per}. 눌러서 확인해 주세요.`,
  };
}

async function push(tokensRef, tokens, msg, link, tag) {
  const entries = Object.entries(tokens || {});
  const bad = [];
  await Promise.all(entries.map(async ([k, token]) => {
    if (typeof token !== "string") return;
    try {
      await admin.messaging().send({
        token,
        notification: { title: msg.title, body: msg.body },
        data: { link, tag },
        webpush: {
          fcmOptions: { link },
          notification: { icon: ICON, badge: ICON, tag, renotify: true, requireInteraction: true, vibrate: [300, 150, 300, 150, 600] },
          headers: { Urgency: "high", TTL: "86400" },
        },
      });
    } catch (e) {
      const c = e && e.code || "";
      if (/registration-token-not-registered|invalid-registration-token|invalid-argument/.test(c)) bad.push(k);
      else console.error("push 실패", c, e && e.message);
    }
  }));
  // 앱을 지운 폰 등 더는 못 받는 주소는 정리
  await Promise.all(bad.map((k) => tokensRef.child(k).remove()));
  return entries.length - bad.length;
}

exports.leakAlert = onValueCreated(
  { ref: "/leak/{dev}/alerts/{aid}", instance: INSTANCE, region: REGION },
  async (ev) => {
    const a = ev.data.val() || {};
    const { dev, aid } = ev.params;
    const db = admin.database();
    const [ownerS, tokS, admS] = await Promise.all([
      db.ref(`leak/${dev}/owner`).get(),
      db.ref(`leak/${dev}/tokens`).get(),
      db.ref("leakAdmin/tokens").get(),
    ]);
    const owner = ownerS.val() || {};
    const msg = words(a, owner);
    const tag = `leak-${dev}`;
    const custLink = `${APP}?go=leakalert&dev=${dev}&a=${aid}`;
    const admLink = `${APP}leak-admin.html?dev=${dev}`;
    const n1 = a.learning ? 0 : await push(db.ref(`leak/${dev}/tokens`), tokS.val(), msg, custLink, tag);
    const n2 = await push(db.ref("leakAdmin/tokens"), admS.val(), {
      title: msg.title, body: `${owner.name || ""} ${owner.phone || ""} · ${msg.body}`.trim(),
    }, admLink, tag);
    await db.ref(`leakIndex/${dev}`).update({ last: { type: a.type || "", t: a.t || Math.floor(Date.now() / 1000), aid, learning: !!a.learning } });
    await ev.data.ref.update({ sent: { cust: n1, adm: n2, at: Date.now() } });
  }
);

exports.leakOffline = onSchedule(
  { schedule: "every 30 minutes", region: REGION, timeZone: "Asia/Seoul" },
  async () => {
    const db = admin.database();
    const idx = (await db.ref("leakIndex").get()).val() || {};
    const now = Math.floor(Date.now() / 1000);
    for (const [dev, v] of Object.entries(idx)) {
      if (!v || !v.seen) continue;
      const off = now - v.seen > 90 * 60;
      if (off && !v.offAt) {
        await db.ref(`leakIndex/${dev}/offAt`).set(now);
        // 장치 대신 서버가 알림을 씀 → leakAlert가 푸시를 보냄
        await db.ref(`leak/${dev}/alerts`).push({ type: "offline", t: now, learning: false });
      } else if (!off && v.offAt) {
        await db.ref(`leakIndex/${dev}/offAt`).remove();
      }
    }
  }
);

// 관리 화면 창구: ?k=관리키&op=list | op=dev&dev=코드 | op=token(POST {token})
exports.leakAdmin = onRequest({ region: REGION, cors: ["https://cdh7657.github.io"] }, async (req, res) => {
  const db = admin.database();
  const key = (await db.ref("leakAdmin/key").get()).val();
  if (!key || String(req.query.k || "") !== String(key)) { res.status(403).json({ err: "key" }); return; }
  const op = req.query.op;
  if (op === "list") { res.json((await db.ref("leakIndex").get()).val() || {}); return; }
  if (op === "dev") {
    const dev = String(req.query.dev || "").replace(/[^A-Z0-9]/g, "");
    if (dev.length !== 10) { res.status(400).json({ err: "dev" }); return; }
    const [st, cfg, owner, al, hist] = await Promise.all([
      db.ref(`leak/${dev}/st`).get(), db.ref(`leak/${dev}/cfg`).get(), db.ref(`leak/${dev}/owner`).get(),
      db.ref(`leak/${dev}/alerts`).orderByKey().limitToLast(20).get(),
      db.ref(`leak/${dev}/hist`).orderByKey().limitToLast(48).get(),
    ]);
    res.json({ st: st.val(), cfg: cfg.val(), owner: owner.val(), alerts: al.val(), hist: hist.val() }); return;
  }
  if (op === "token" && req.method === "POST") {
    const t = String((req.body && req.body.token) || "");
    if (t.length < 20) { res.status(400).json({ err: "token" }); return; }
    let h = 0; for (let i = 0; i < t.length; i++) h = (h * 31 + t.charCodeAt(i)) | 0;
    await db.ref(`leakAdmin/tokens/t${(h >>> 0).toString(36)}${t.length}`).set(t);
    res.json({ ok: true }); return;
  }
  if (op === "cfg" && req.method === "POST") {  // 기준값 조정(riseBar, flowBar, flowMin, mode, learnUntil)
    const dev = String(req.query.dev || "").replace(/[^A-Z0-9]/g, "");
    const b = req.body || {}, up = {};
    ["riseBar", "flowBar", "flowMin", "learnUntil", "base"].forEach((k) => { if (typeof b[k] === "number") up[k] = b[k]; });
    if (b.mode === "prv" || b.mode === "basic") up.mode = b.mode;
    if (dev.length !== 10 || !Object.keys(up).length) { res.status(400).json({ err: "cfg" }); return; }
    await db.ref(`leak/${dev}/cfg`).update(up); res.json({ ok: true, up }); return;
  }
  res.status(400).json({ err: "op" });
});
