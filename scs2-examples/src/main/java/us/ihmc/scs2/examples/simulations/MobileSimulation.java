package us.ihmc.scs2.examples.simulations;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;

public class MobileSimulation
{
   public static void main(String[] args)
   {
      MobileDefinition definition = new MobileDefinition();

      SimulationSession simulationSession = new SimulationSession();
      simulationSession.addRobot(definition);

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }
}
