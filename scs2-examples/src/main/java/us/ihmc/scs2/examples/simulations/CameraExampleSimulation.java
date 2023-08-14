package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraControlMode;
import us.ihmc.yoVariables.variable.YoDouble;

public class CameraExampleSimulation
{
   private static final CameraControlMode controlModeToBind = CameraControlMode.Orbital;

   public static void main(String[] args)
   {
      BoxRobotDefinition definition = new BoxRobotDefinition();
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      definition.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory());
      scs.addRobot(definition);
      scs.addTerrainObject(new SlopeGroundDefinition());

      YoDouble cameraX = new YoDouble("cameraX", scs.getRootRegistry());
      YoDouble cameraY = new YoDouble("cameraY", scs.getRootRegistry());
      YoDouble cameraZ = new YoDouble("cameraZ", scs.getRootRegistry());
      YoDouble cameraDistance = new YoDouble("cameraDistance", scs.getRootRegistry());
      YoDouble cameraLongitude = new YoDouble("cameraLongitude", scs.getRootRegistry());
      YoDouble cameraLatitude = new YoDouble("cameraLatitude", scs.getRootRegistry());

      cameraX.set(1.0);
      cameraY.set(3.0);
      cameraZ.set(0.5);

      cameraDistance.set(6.0);
      cameraLongitude.set(0.5 * Math.PI);
      cameraLatitude.set(0.5 * Math.PI);

      scs.addAfterPhysicsCallback(time ->
      {
         cameraX.set(2.0 + 0.25 * Math.sin(2.0 * Math.PI / 2.0 * time + Math.toRadians(30)));
         cameraY.set(3.5 + 0.25 * Math.sin(2.0 * Math.PI / 2.0 * time + Math.toRadians(60)));
         cameraZ.set(1.0 + 0.25 * Math.sin(2.0 * Math.PI / 2.0 * time + Math.toRadians(90)));

         cameraDistance.set(5.0 + 0.5 * (1 + Math.sin(2.0 * Math.PI / 2.0 * time + Math.toRadians(30))));
         cameraLongitude.set(Math.toRadians(180) + Math.toRadians(90) * Math.sin(2.0 * Math.PI / 10.0 * time + Math.toRadians(60)));
         cameraLatitude.set(Math.toRadians(30.0) * (1 + Math.sin(2.0 * Math.PI / 2.0 * time + Math.toRadians(30))));
      });

      scs.requestCameraRigidBodyTracking(definition.getName(), definition.getFloatingRootJointDefinition().getSuccessor().getName());
      if (controlModeToBind == CameraControlMode.Position)
         scs.requestCameraPositionTracking(cameraX, cameraY, cameraZ);
      else if (controlModeToBind == CameraControlMode.Orbital)
         scs.requestCameraOrbitTracking(cameraDistance, cameraLongitude, cameraLatitude);
      else if (controlModeToBind == CameraControlMode.LevelOrbital)
         scs.requestCameraLevelOrbitTracking(cameraDistance, cameraLongitude, cameraZ);

      scs.initializeBufferSize(100000);
      scs.setRealTimeRateSimulation(true);
      scs.start(true, false, false);
   }
}
