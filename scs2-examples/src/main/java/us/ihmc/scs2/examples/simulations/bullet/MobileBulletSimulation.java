package us.ihmc.scs2.examples.simulations.bullet;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyJointParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletMultiBodyParameters;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;

public class MobileBulletSimulation
{
   public static void main(String[] args)
   {
      MobileBulletDefinition definition = new MobileBulletDefinition();

      BulletMultiBodyParameters parameters = BulletMultiBodyParameters.defaultBulletMultiBodyParameters();
      parameters.setFixedBase(true);
      parameters.setHasSelfCollision(false);
      parameters.setAngularDamping(0.0);
      parameters.setLinearDamping(0.0);
      BulletMultiBodyJointParameters jointParameters = BulletMultiBodyJointParameters.defaultBulletMultiBodyJointParameters();
//      jointParameters.setJointFriction(0.2);

      SimulationSession simulationSession = new SimulationSession((frame,
                                                                   rootRegistry) -> new BulletPhysicsEngine(frame,
                                                                                                            rootRegistry,
                                                                                                            parameters,
                                                                                                            jointParameters));

      simulationSession.addRobot(definition);

      //SessionVisualizer.startSessionVisualizer(simulationSession);

      SessionVisualizer sessionVisualizer = BulletExampleSimulationTools.startSessionVisualizerWithDebugDrawing(simulationSession);

      sessionVisualizer.getSessionVisualizerControls().setCameraFocusPosition(0.0, 0.0, 0.7);
//      sessionVisualizer.getToolkit().getSession().runTick();
   }
}
