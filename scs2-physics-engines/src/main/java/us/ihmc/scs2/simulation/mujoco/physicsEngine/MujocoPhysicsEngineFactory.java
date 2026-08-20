package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParameters;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSimulationParametersReadOnly;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface MujocoPhysicsEngineFactory
{
   PhysicsEngine build(ReferenceFrame inertialFrame, YoRegistry rootRegistry);

   static PhysicsEngineFactory newMujocoPhysicsEngineFactory()
   {
      return newMujocoPhysicsEngineFactory(MujocoSimulationParameters.defaultMujocoSimulationParameters());
   }

   static PhysicsEngineFactory newMujocoPhysicsEngineFactory(MujocoSimulationParametersReadOnly parameters)
   {
      return (frame, rootRegistry) -> new MujocoPhysicsEngine(frame, rootRegistry, parameters);
   }
}
