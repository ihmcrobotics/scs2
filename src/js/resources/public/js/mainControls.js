var webSocket = new WebSocket(
  "ws://" + location.hostname + ":" + location.port + "/maincontrols"
);

var simulateButton = document.getElementById("simulateButton");
simulateButton.addEventListener("click", function () {
  webSocket.send("requestSimulate");
});

var playButton = document.getElementById("playButton");
playButton.addEventListener("click", function () {
  webSocket.send("requestPlayback");
});

var pauseButton = document.getElementById("pauseButton");
pauseButton.addEventListener("click", function () {
  webSocket.send("requestPause");
});
