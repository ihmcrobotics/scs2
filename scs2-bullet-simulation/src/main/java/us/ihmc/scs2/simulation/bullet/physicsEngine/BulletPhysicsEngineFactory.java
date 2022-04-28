package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface BulletPhysicsEngineFactory
{
   PhysicsEngine build(ReferenceFrame inertialFrame, YoRegistry rootRegistry);

   static PhysicsEngineFactory newBulletPhysicsEngineFactory()
   {
      return (frame, rootRegistry) ->
      {
         BulletPhysicsEngine physicsEngine = new BulletPhysicsEngine(frame, rootRegistry);
         return physicsEngine;
      };
   }

   static PhysicsEngineFactory newBulletPhysicsEngineFactory(BulletMultiBodyParameters bulletMultiBodyParameters, BulletMultiBodyJointParameters bulletMultiBodyJointParameters)
   {
      return (frame, rootRegistry) ->
      {
         BulletPhysicsEngine physicsEngine = new BulletPhysicsEngine(frame, rootRegistry);
         physicsEngine.setGlobalBulletMultiBodyParameters(bulletMultiBodyParameters);
         physicsEngine.setGlobalBulletMultiBodyJointParameters(bulletMultiBodyJointParameters);
         return physicsEngine;
      };
   }
}
