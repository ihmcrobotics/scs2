import * as THREE from "https://cdnjs.cloudflare.com/ajax/libs/three.js/110/three.module.js";
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@v0.108.0/examples/jsm/controls/OrbitControls.js";
import * as ProtoTools from "./prototools.js";

THREE.Object3D.DefaultUp = new THREE.Vector3(0, 0, 1);

var webSocket = new WebSocket(
  "ws://" + location.hostname + ":" + location.port + "/viewport"
);

webSocket.onclose = function (event) {
  if (event.wasClean) {
    alert(
      "[close] Connection closed cleanly, code=${event.code} reason=${event.reason}"
    );
  } else {
    // e.g. server process killed or network down
    // event.code is usually 1006 in this case
    alert("[close] Connection died code=" + event.code);
  }
};

var canvas = document.getElementById("mainViewportCanvas");

var scene = new THREE.Scene();
const loader = new THREE.CubeTextureLoader();
scene.background = loader
  .setPath("./skybox/cloudy/")
  .load(
    ["Back.png", "Front.png", "Left.png", "Right.png", "Up.png", "Down.png"],
    function (texture) {
      console.info(texture);
    }
  );
var camera = new THREE.PerspectiveCamera(75, 2, 0.1, 1000.0);
camera.position.z = 5;

var controls = new OrbitControls(camera, canvas);
controls.update();

var renderer = new THREE.WebGLRenderer({ antialias: true, canvas: canvas });
renderer.setClearColor("#e5e5e5");

var raycaster = new THREE.Raycaster();
var mouse = new THREE.Vector2();

var geometry = new THREE.BoxGeometry(1, 1, 1);

for (let i = 0; i < 15; i++) {
  let material = new THREE.MeshLambertMaterial({ color: 0xf7f7f7 });
  let mesh = new THREE.Mesh(geometry, material);
  mesh.position.x = (Math.random() - 0.5) * 10;
  mesh.position.y = (Math.random() - 0.5) * 10;
  mesh.position.z = (Math.random() - 0.5) * 10;
  scene.add(mesh);
}

var light = new THREE.PointLight(0xffffff, 1, 1000);
light.position.set(0, 0, 0);
scene.add(light);

var light = new THREE.PointLight(0xffffff, 2, 1000);
light.position.set(0, 0, 25);
scene.add(light);

function resizeCanvasToDisplaySize() {
  // look up the size the canvas is being displayed
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;

  // adjust displayBuffer size to match
  if (canvas.width !== width || canvas.height !== height) {
    // you must pass false here or three.js sadly fights the browser
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

function animate(time) {
  resizeCanvasToDisplaySize();

  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

// function onMouseMove(event) {
//   event.preventDefault();

//   var localPosition = getMousePosition(canvas, event);
//   mouse.x = (localPosition.x / canvas.width) * 2 - 1;
//   mouse.y = -(localPosition.y / canvas.height) * 2 + 1;

//   raycaster.setFromCamera(mouse, camera);

//   var intersects = raycaster.intersectObjects(scene.children, true);
//   for (var i = 0; i < intersects.length; i++) {
//     var target = intersects[i].object;
//     target.material.color.set(0xffff00);
//     this.tl = new TimelineMax().delay(0.3);
//     this.tl.to(target.scale, 1, { x: 2, ease: Expo.easeOut });
//     this.tl.to(target.scale, 0.5, { x: 0.5, ease: Expo.easeOut });
//     this.tl.to(target.position, 0.5, { x: 2.0, ease: Expo.easeOut });
//   }
// }

// function getMousePosition(canvas, event) {
//   var rect = canvas.getBoundingClientRect();
//   return {
//     x: event.clientX - rect.left,
//     y: event.clientY - rect.top,
//   };
// }
// canvas.addEventListener("click", onMouseMove);

requestAnimationFrame(animate);

var meshGroupMap = new Map();

webSocket.onmessage = function (event) {
  event.data.arrayBuffer().then((value) => {
    var protoMeshGroup = proto.protobuf.MeshGroup.deserializeBinary(value);
    console.info(protoMeshGroup);
    var meshGroup;

    if (!meshGroupMap.has(protoMeshGroup.getGroupId())) {
      meshGroup = new THREE.Group();
      protoMeshGroup.getMeshesList().forEach((protoMesh) => {
        let newMesh = ProtoTools.toTHREEMesh(protoMesh);
        console.info(newMesh);
        meshGroup.add(newMesh);
      });
      scene.add(meshGroup);
      meshGroupMap.set(protoMeshGroup.getGroupId(), meshGroup);
      console.info(meshGroupMap);
    } else {
      meshGroup = meshGroupMap.get(protoMeshGroup.getGroupId());
    }

    if (protoMeshGroup.hasPose()) {
      ProtoTools.packTHREEPose(
        protoMeshGroup.getPose(),
        meshGroup.quaternion,
        meshGroup.position
      );
    }
  });
};
