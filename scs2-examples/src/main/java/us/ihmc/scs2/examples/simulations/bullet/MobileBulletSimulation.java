package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public class MobileBulletSimulation
{
   private static final boolean VISUALIZE_WITH_DEBUG_DRAWING = false;
   private static YoDouble realtimeRate;

   public static void main(String[] args)
   {
      MobileBulletDefinition definition = new MobileBulletDefinition();

      BulletMultiBodyParameters parameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      parameters.setHasSelfCollision(false);
      BulletMultiBodyJointParameters jointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(parameters, jointParameters));
      simulationSession.addRobot(definition);

      realtimeRate = new YoDouble("realtimeRate", simulationSession.getRootRegistry());



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

//         sessionVisualizerControls.

//         ThreadTools.


      }
   }
}
