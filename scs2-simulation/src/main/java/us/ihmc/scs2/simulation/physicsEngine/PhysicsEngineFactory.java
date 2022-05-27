package us.ihmc.scs2.simulation.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface PhysicsEngineFactory
{
   PhysicsEngine build(ReferenceFrame inertialFrame, YoRegistry rootRegistry);

   static PhysicsEngineFactory newImpulseBasedPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new ImpulseBasedPhysicsEngine(frame, rootRegistry);
   }

   static PhysicsEngineFactory newImpulseBasedPhysicsEngineFactory(ContactParametersReadOnly contactParameters)
   {
      return (frame, rootRegistry) ->
      {
         ImpulseBasedPhysicsEngine physicsEngine = new ImpulseBasedPhysicsEngine(frame, rootRegistry);
         if (contactParameters != null)
            physicsEngine.setGlobalContactParameters(contactParameters);
         return physicsEngine;
      };
   }
}
