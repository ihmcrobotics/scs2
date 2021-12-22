package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;

public class FallingBallSimulation
{
   public static void main(String[] args)
   {
      BallRobotDefinition definition = new BallRobotDefinition();
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      initialJointState.setVelocity(new Vector3D(10.0, 0.0, 0.0), new Vector3D(-1.0, 0.0, 0.0));
      definition.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      SimulationSession simulationSession = new SimulationSession();
      simulationSession.addRobot(definition);
      simulationSession.addTerrainObject(new SlopeGroundDefinition(Math.toRadians(15.0)));

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }
}
