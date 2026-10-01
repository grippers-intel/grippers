/* vla_robot run_host <-> 팀원 시연 UI(grippers-ui.html) 연결.
 *
 * 원본 UI 는 pywebview 창(ui_bridge.DemoUI)에 붙도록 만들어졌다 — 파이썬이
 * window.applyHostState(state) 를 부르고, 버튼은 pywebview.api.ui_event(action, payload)
 * 로 돌아간다. 여기서는 pywebview 대신 run_host 의 로컬 HTTP 서버를 쓴다(윈도우·맥 모두
 * 추가 설치 없이 브라우저만 있으면 된다):
 *   - GET  /state  -> 최신 state(JSON)를 0.1 s 마다 받아 applyHostState 로 넘긴다
 *   - POST /event  -> {action, payload}
 * 이 파일은 </head> 앞에 들어가므로 UI 스크립트보다 먼저 돈다.
 */
(function () {
  "use strict";
  // 목업 재생을 끈다(app.js boot() 가 0.6 s 뒤에 이 값을 본다).
  window.__hostAttached = true;

  // app.js 의 send() 가 찾는 자리. 응답은 기다리지 않는다.
  window.pywebview = {
    api: {
      ui_event: function (action, payload) {
        fetch("/event", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ action: action, payload: payload == null ? null : payload }),
        }).catch(function () { /* 서버가 내려가면 조용히 */ });
      },
    },
  };

  // q = run_host 종료(예전 OpenCV 지도와 같은 키). 원래 UI 에는 종료 키가 없다.
  // 입력창에 타이핑하는 중에는 가로채지 않는다(app.js 단축키와 같은 규칙).
  window.addEventListener("keydown", function (e) {
    var t = e.target;
    var typing = t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.isContentEditable);
    if (!typing && (e.key === "q" || e.key === "Q") && !e.ctrlKey && !e.metaKey && !e.altKey) {
      window.pywebview.api.ui_event("quit", null);
    }
  });

  var failures = 0;
  function poll() {
    fetch("/state", { cache: "no-store" })
      .then(function (r) { return r.ok ? r.json() : null; })
      .then(function (s) {
        failures = 0;
        if (s && window.applyHostState) window.applyHostState(s);
      })
      .catch(function () {
        // run_host 가 끝났다 — 화면에 한 줄 남기고 느리게 다시 시도한다.
        failures += 1;
        if (failures === 20 && window.showBootError) {
          window.showBootError("run_host 와 연결이 끊겼습니다 (Ctrl+C 로 종료했다면 이 창을 닫으세요)");
        }
      })
      .then(function () { setTimeout(poll, failures ? 500 : 100); });
  }
  document.addEventListener("DOMContentLoaded", poll);
})();
