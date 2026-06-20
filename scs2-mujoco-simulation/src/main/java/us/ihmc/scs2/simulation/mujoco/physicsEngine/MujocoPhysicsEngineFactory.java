package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParametersReadOnly;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

/**
 * Static factories that produce {@link PhysicsEngineFactory} lambdas wired to {@link MujocoPhysicsEngine}.
 * Pass the returned factory to {@code SimulationConstructionSet2} or {@code SCS2JavaFXVisualizer}
 * wherever a {@link PhysicsEngineFactory} is accepted.
 */
public interface MujocoPhysicsEngineFactory
{
   static PhysicsEngineFactory newMujocoPhysicsEngineFactory(MujocoSimulationParametersReadOnly parameters)
   {
      return (frame, rootRegistry) -> new MujocoPhysicsEngine(frame, rootRegistry, parameters);
   }
}
