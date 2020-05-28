import * as THREE from "https://cdnjs.cloudflare.com/ajax/libs/three.js/110/three.module.js";

export function packTHREEVector2(protoVector2D, threeVector2ToPack) {
  threeVector2ToPack.x = protoVector2D.getX();
  threeVector2ToPack.y = protoVector2D.getY();
}

export function packTHREEVector3(protoVector3D, threeVector3ToPack) {
  threeVector3ToPack.x = protoVector3D.getX();
  threeVector3ToPack.y = protoVector3D.getY();
  threeVector3ToPack.z = protoVector3D.getZ();
}

export function packTHREEEuler(protoYawPitchRoll, threeEulerToPack) {
  threeEulerToPack.x = protoYawPitchRoll.getRoll();
  threeEulerToPack.y = protoYawPitchRoll.getPitch();
  threeEulerToPack.z = protoYawPitchRoll.getYaw();
  threeEulerToPack.order = "ZYZ";
}

export function packTHREEQuaternion(protoQuaternion, threeQuaternionToPack) {
  threeQuaternionToPack.x = protoQuaternion.getX();
  threeQuaternionToPack.y = protoQuaternion.getY();
  threeQuaternionToPack.z = protoQuaternion.getZ();
  threeQuaternionToPack.w = protoQuaternion.getS();
}

export function packTHREEPose(
  protoPose3D,
  threeQuaternionToPack,
  threePositionToPack
) {
  if (protoPose3D.hasOrientation()) {
    packTHREEQuaternion(protoPose3D.getOrientation(), threeQuaternionToPack);
  } else {
    threeQuaternionToPack.set(0, 0, 0, 1);
  }

  if (protoPose3D.hasPosition()) {
    packTHREEVector3(protoPose3D.getPosition(), threePositionToPack);
  } else {
    threePositionToPack.set(0, 0, 0);
  }
}

export function toTHREEColor(protoColor) {
  var color = w3color(protoColor.getWebcolor()).toRgb();
  return new THREE.Color(color.r / 255.0, color.g / 255.0, color.b / 255.0);
}

export function toTHREEMaterial(protoColor) {
  var color = w3color(protoColor.getWebcolor()).toRgb();
  if (color.a > 0.99) {
    return new THREE.MeshLambertMaterial({
      color: toTHREEColor(protoColor),
    });
  } else {
    return new THREE.MeshLambertMaterial({
      color: toTHREEColor(protoColor),
      opacity: color.a,
      transparent: true,
    });
  }
}

export function toTHREEBoxGeometry(protoGeometry) {
  let threeGeometry = new THREE.BoxGeometry(
    protoGeometry.getSize().getX(),
    protoGeometry.getSize().getY(),
    protoGeometry.getSize().getZ()
  );
  return threeGeometry;
}

export function toTHREEConeGeometry(protoGeometry) {
  let threeGeometry = new THREE.ConeGeometry(
    protoGeometry.getRadius(),
    protoGeometry.getHeight(),
    protoGeometry.getRadialSegments()
  );
  return threeGeometry;
}

export function toTHREECylinderGeometry(protoGeometry) {
  let threeGeometry = new THREE.CylinderGeometry(
    protoGeometry.getRadius(),
    protoGeometry.getHeight(),
    protoGeometry.getRadialSegments()
  );
  return threeGeometry;
}

export function toTHREESphereGeometry(protoGeometry) {
  let threeGeometry = new THREE.SphereGeometry(
    protoGeometry.getRadius(),
    protoGeometry.getWidthSegments(),
    protoGeometry.getHeightSegments()
  );
  return threeGeometry;
}

export function toTHREEMesh(protoMesh) {
  let geometry;
  if (protoMesh.hasBoxgeometry()) {
    geometry = toTHREEBoxGeometry(protoMesh.getBoxgeometry());
  } else if (protoMesh.hasConegeometry()) {
    geometry = toTHREEConeGeometry(protoMesh.getConegeometry());
  } else if (protoMesh.hasCylindergeometry()) {
    geometry = toTHREECylinderGeometry(protoMesh.getCylindergeometry());
  } else if (protoMesh.hasSpheregeometry()) {
    geometry = toTHREESphereGeometry(protoMesh.getSpheregeometry());
  } else if (protoMesh.hasModelfilegeometry()) {
    console.error("Unsupport model file.");
    return;
  }

  let material = toTHREEMaterial(protoMesh.getColor());
  let mesh = new THREE.Mesh(geometry, material);

  if (protoMesh.hasPose()) {
    packTHREEPose(protoMesh.getPose(), mesh.quaternion, mesh.position);
  }
  return mesh;
}
