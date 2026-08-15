package us.ihmc.scs2.simulation.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Functional interface for creating a new physics engine to be used in a simulation session.
 * <p>
 * This interface only provides the trivial {@link #newDoNothingPhysicsEngineFactory()} factory,
 * which has no dependency beyond this module. Factories for the concrete physics engines
 * (contact-point-based, impulse-based, Bullet, MuJoCo) live in {@code scs2-physics-engine-implementation}'s
 * {@code PhysicsEngineFactories}, along with the {@link PhysicsEngineType}-based resolver -- that
 * module is the one place allowed to reference all concrete engine implementations at compile time.
 * </p>
 *
 * @see SimulationSession
 * @see PhysicsEngineType
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

   static PhysicsEngineFactory newDoNothingPhysicsEngineFactory()
   {
      return (frame, rootRegistry) -> new DoNothingPhysicsEngine(frame, rootRegistry);
   }
}
