package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.commons.Conversions;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.parameters.BulletMultiBodyParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MobileBulletSimulation
{
   private static final boolean VISUALIZE_WITH_DEBUG_DRAWING = false;
   private static final double DT = 1.0 / 250.0;

   public static SimulationSession createSession()
   {
      MobileBulletDefinition definition = new MobileBulletDefinition();
      definition.addControllerDefinition((controllerInput, controllerOutput) -> new Controller()
      {
         YoRegistry registry = new YoRegistry("MobileBulletRobotController");
         YoDouble realtimeRate = new YoDouble("realtimeRate", registry);
         double lastRecord = Double.NaN;

         @Override
         public void initialize()
         {

         }

         @Override
         public void doControl()
         {
            double currentRecord = Conversions.nanosecondsToSeconds(System.nanoTime());

            if (!Double.isNaN(lastRecord))
            {
               realtimeRate.set(DT / (currentRecord - lastRecord));
            }

            lastRecord = currentRecord;
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return registry;
         }
      });

      BulletMultiBodyParameters parameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      parameters.setHasSelfCollision(false);
      BulletMultiBodyJointParameters jointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(parameters, jointParameters));
      simulationSession.addRobot(definition);
      simulationSession.setSessionDTSeconds(DT);
      return simulationSession;
   }

   public static void main(String[] args)
   {
      SimulationSession simulationSession = createSession();
      if (VISUALIZE_WITH_DEBUG_DRAWING)
      {
         SessionVisualizer sessionVisualizer = BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);
         sessionVisualizer.getSessionVisualizerControls().setCameraFocalPosition(0.0, 0.0, 0.7);
         sessionVisualizer.getToolkit().getSession().runTick();
      }
      else
      {
         SessionVisualizerControls sessionVisualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession, null);
         sessionVisualizerControls.setCameraFocalPosition(0.0, 0.0, 0.7);
      }
   }
}
