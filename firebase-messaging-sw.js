/* 누수알림기 푸시 알림 받기 (고객앱·관리 화면 공용)
   앱이 꺼져 있어도 휴대폰에 알림을 띄우고, 누르면 확인 화면을 연다. 화면 캐시는 하지 않음(앱 업데이트에 영향 없음). */
/* 알림을 누르면: 이미 열린 앱 창이 있으면 그 창에서, 없으면 새로 열기 (Firebase보다 먼저 등록) */
self.addEventListener("notificationclick", function (e) {
  var d = (e.notification && e.notification.data) || {};
  var link = (d.FCM_MSG && d.FCM_MSG.data && d.FCM_MSG.data.link) || d.link || "https://cdh7657.github.io/nusu/";
  e.stopImmediatePropagation();  // Firebase 기본 처리와 겹쳐 창이 두 번 열리지 않게
  e.notification.close();
  e.waitUntil(clients.matchAll({ type: "window", includeUncontrolled: true }).then(function (ws) {
    for (var i = 0; i < ws.length; i++) { if (ws[i].url.indexOf("/nusu/") >= 0 && "navigate" in ws[i]) { return ws[i].navigate(link).then(function (w) { return w && w.focus(); }); } }
    return clients.openWindow(link);
  }));
});

importScripts("https://www.gstatic.com/firebasejs/13.2.0/firebase-app-compat.js");
importScripts("https://www.gstatic.com/firebasejs/13.2.0/firebase-messaging-compat.js");
firebase.initializeApp({
  apiKey: "AIzaSyB8NWjchZimJNbX1Nn_bNcIW234afMWDeE",
  authDomain: "daewon-booking.firebaseapp.com",
  projectId: "daewon-booking",
  storageBucket: "daewon-booking.firebasestorage.app",
  messagingSenderId: "295193453425",
  appId: "1:295193453425:web:86f0e1a695af830ab14cd0",
});
firebase.messaging();  // notification 이 담긴 푸시는 자동으로 알림을 띄움

