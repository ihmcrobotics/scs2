package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;

public class MobileBulletSimulation
{
   private static final boolean DEBUG = false;
   
   public static void main(String[] args)
   {
      MobileBulletDefinition definition = new MobileBulletDefinition();

      BulletMultiBodyParameters parameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      parameters.setFixedBase(true);
      parameters.setHasSelfCollision(false);
      parameters.setAngularDamping(0.04);
      parameters.setLinearDamping(0.04);
      BulletMultiBodyJointParameters jointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
      jointParameters.setJointFriction(0.0);
      jointParameters.setJointContactProcessingThreshold(0);

      SimulationSession simulationSession = new SimulationSession(BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory(parameters, jointParameters));

      simulationSession.addRobot(definition);
      if (!DEBUG)
         SessionVisualizer.startSessionVisualizer(simulationSession);
      else
      {
         SessionVisualizer sessionVisualizer = BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);
         sessionVisualizer.getSessionVisualizerControls().setCameraFocusPosition(0.0, 0.0, 0.7);
         sessionVisualizer.getToolkit().getSession().runTick();
      }
   }
}
