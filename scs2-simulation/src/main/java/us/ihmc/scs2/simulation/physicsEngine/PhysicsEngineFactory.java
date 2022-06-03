package us.ihmc.scs2.simulation.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParametersReadOnly;
import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Functional interface for creating a new physics engine to be used in a simulation session.
 * <p>
 * This interface provides 2 default factories:
 * <ul>
 * <li>{@link #newContactPointBasedPhysicsEngineFactory()}: for setting up a contact point based
 * physics engine. This is an adaption from SCS1 physics engine. Only simulates point to shape
 * contacts, contacts and joint limits are enforced using soft constraints.
 * <li>{@link #newImpulseBasedPhysicsEngineFactory()}: for setting up an impulse based physics
 * engine. This physics engine is still at the experimental phase. Shape to shape contacts can be
 * simulated, contact and joint limits are resolved as hard constraints.
 * </ul>
 * </p>
 * 
 * @see SimulationSession
 * @author Sylvain Bertrand
 */
public interface PhysicsEngineFactory
{
   /**
    * Creates the physics engine to be used in a simulation session.
    * 
    * @param inertialFrame the root frame used for this session. It is typically different from
    *                      {@link ReferenceFrame#getWorldFrame()}.
    * @param rootRegistry  the session's root registry for registering robot state variables for
    *                      instance.
    * @return the new physics engine.
    */
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

   static PhysicsEngineFactory newContactPointBasedPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new ContactPointBasedPhysicsEngine(frame, rootRegistry);
   }

   static PhysicsEngineFactory newContactPointBasedPhysicsEngineFactory(ContactPointBasedContactParametersReadOnly contactParameters)
   {
      return (frame, rootRegistry) ->
      {
         ContactPointBasedPhysicsEngine physicsEngine = new ContactPointBasedPhysicsEngine(frame, rootRegistry);
         if (contactParameters != null)
            physicsEngine.setGroundContactParameters(contactParameters);
         return physicsEngine;
      };
   }

   static PhysicsEngineFactory newDoNothingPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new DoNothingPhysicsEngine(frame, rootRegistry);
   }
}
