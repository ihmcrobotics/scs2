package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;

public class MobileBulletSimulation
{
   private static final boolean VISUALIZE_WITH_DEBUG_DRAWING = false;
   
   public static void main(String[] args)
   {
      MobileBulletDefinition definition = new MobileBulletDefinition();

      BulletMultiBodyParameters parameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      parameters.setHasSelfCollision(false);
      BulletMultiBodyJointParameters jointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(parameters, jointParameters));
      simulationSession.addRobot(definition);
      
      if (VISUALIZE_WITH_DEBUG_DRAWING)
      {
         SessionVisualizer sessionVisualizer = BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);
         sessionVisualizer.getSessionVisualizerControls().setCameraFocusPosition(0.0, 0.0, 0.7);
         sessionVisualizer.getToolkit().getSession().runTick();
      }
      else
      {
         SessionVisualizerControls sessionVisualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession, null);
         sessionVisualizerControls.setCameraFocusPosition(0.0, 0.0, 0.7);
      }
   }
}
