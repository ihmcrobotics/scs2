package us.ihmc.scs2.examples.simulations;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.PhysicsEngineFactories;

public class MobileSimulation
{
   public static void main(String[] args)
   {
      MobileDefinition definition = new MobileDefinition();

      SimulationSession simulationSession = new SimulationSession(PhysicsEngineFactories.newImpulseBasedPhysicsEngineFactory());
      simulationSession.addRobot(definition);

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }
}
