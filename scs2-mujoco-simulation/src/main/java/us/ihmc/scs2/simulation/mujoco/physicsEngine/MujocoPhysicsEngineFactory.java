package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

/**
 * Static factories that produce {@link PhysicsEngineFactory} lambdas wired to {@link MujocoPhysicsEngine}.
 * Use exactly as you would {@code BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory()}.
 */
public interface MujocoPhysicsEngineFactory
{
   static PhysicsEngineFactory newMujocoPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new MujocoPhysicsEngine(frame, rootRegistry);
   }

   static PhysicsEngineFactory newMujocoPhysicsEngineFactory(MujocoSimulationParameters parameters)
   {
      return (frame, rootRegistry) -> new MujocoPhysicsEngine(frame, rootRegistry, parameters);
   }
}
