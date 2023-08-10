package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.yoVariables.variable.YoDouble;

public class CameraExampleSimulation
{
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

      scs.initializeBufferSize(100000);
      scs.start(true, false, false);
   }
}
