package us.ihmc.scs2.simulation.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface PhysicsEngineFactory
{
   PhysicsEngine build(ReferenceFrame inertialFrame, YoRegistry rootRegistry);

   static PhysicsEngineFactory newImpulseBasedPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new ImpulseBasedPhysicsEngine(frame, rootRegistry);
   }

   static PhysicsEngineFactory newImpulseBasedPhysicsEngineFactory(ContactParameters contactParameters)
   {
      return (frame, rootRegistry) ->
      {
         ImpulseBasedPhysicsEngine physicsEngine = new ImpulseBasedPhysicsEngine(frame, rootRegistry);
         physicsEngine.setGlobalContactParameters(contactParameters);
         return physicsEngine;
      };
   }
}
