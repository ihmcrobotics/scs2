package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;

public class MobileBulletSimulation
{
   public static void main(String[] args)
   {
      MobileBulletDefinition definition = new MobileBulletDefinition();

      SimulationSession simulationSession = new SimulationSession((frame, rootRegistry) -> new BulletPhysicsEngine(frame, rootRegistry, BulletMultiBodyParameters.defaultBulletMultiBodyParameters()));
      simulationSession.addRobot(definition);

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }
}
