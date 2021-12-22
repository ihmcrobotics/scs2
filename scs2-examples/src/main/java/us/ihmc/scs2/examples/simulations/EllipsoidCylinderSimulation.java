package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;

public class EllipsoidCylinderSimulation
{
   public static void main(String[] args)
   {
      EllipsoidRobotSimulation definition = new EllipsoidRobotSimulation();
      definition.getRootJointDefinitions().get(0).setInitialJointState(new SixDoFJointState(new YawPitchRoll(0, 0, 0.6), new Point3D(0, 0, 1.0)));

      SimulationSession simulationCore = new SimulationSession();
      simulationCore.addRobot(definition);
      simulationCore.addTerrainObject(new SlopeGroundDefinition());

      SessionVisualizer.startSessionVisualizer(simulationCore);
   }
}
