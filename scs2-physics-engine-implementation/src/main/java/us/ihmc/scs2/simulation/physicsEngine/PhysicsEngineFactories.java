package us.ihmc.scs2.simulation.physicsEngine;

import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngineFactory;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngineFactory;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParametersReadOnly;
import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;

/**
 * Resolves a {@link PhysicsEngineType} to a {@link PhysicsEngineFactory}.
 * <p>
 * This class is the single place in the {@code scs2} repo allowed to reference all four concrete
 * physics engine implementations at compile time -- that's the entire reason the four engines were
 * consolidated into one Gradle module ({@code scs2-physics-engine-implementation}).
 * </p>
 */
public class PhysicsEngineFactories
{
   private PhysicsEngineFactories()
   {
      // static class
   }

   public static PhysicsEngineFactory newPhysicsEngineFactory(PhysicsEngineType type)
   {
      switch (type)
      {
         case CONTACT_POINT_BASED:
            return newContactPointBasedPhysicsEngineFactory();
         case IMPULSE_BASED:
            return newImpulseBasedPhysicsEngineFactory();
         case BULLET:
            return newBulletPhysicsEngineFactory();
         case MUJOCO:
            return newMujocoPhysicsEngineFactory();
         default:
            throw new IllegalArgumentException("Unhandled " + PhysicsEngineType.class.getSimpleName() + ": " + type);
      }
   }

   public static PhysicsEngineFactory newContactPointBasedPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new ContactPointBasedPhysicsEngine(frame, rootRegistry);
   }

   public static PhysicsEngineFactory newContactPointBasedPhysicsEngineFactory(ContactPointBasedContactParametersReadOnly contactParameters)
   {
      return (frame, rootRegistry) ->
      {
         ContactPointBasedPhysicsEngine physicsEngine = new ContactPointBasedPhysicsEngine(frame, rootRegistry);
         if (contactParameters != null)
            physicsEngine.setGroundContactParameters(contactParameters);
         return physicsEngine;
      };
   }

   public static PhysicsEngineFactory newImpulseBasedPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new ImpulseBasedPhysicsEngine(frame, rootRegistry);
   }

   public static PhysicsEngineFactory newImpulseBasedPhysicsEngineFactory(ContactParametersReadOnly contactParameters)
   {
      return (frame, rootRegistry) ->
      {
         ImpulseBasedPhysicsEngine physicsEngine = new ImpulseBasedPhysicsEngine(frame, rootRegistry);
         if (contactParameters != null)
            physicsEngine.setGlobalContactParameters(contactParameters);
         return physicsEngine;
      };
   }

   public static PhysicsEngineFactory newBulletPhysicsEngineFactory()
   {
      return BulletPhysicsEngineFactory.newBulletPhysicsEngineFactory();
   }

   public static PhysicsEngineFactory newMujocoPhysicsEngineFactory()
   {
      return MujocoPhysicsEngineFactory.newMujocoPhysicsEngineFactory();
   }
}
