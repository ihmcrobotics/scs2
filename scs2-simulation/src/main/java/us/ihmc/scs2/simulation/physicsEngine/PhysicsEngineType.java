package us.ihmc.scs2.simulation.physicsEngine;

/**
 * Identifies the physics engine implementation to be used in a simulation session.
 * <p>
 * This enum lives in {@code scs2-simulation} (core) so it has zero dependency on any concrete
 * physics engine or native library. Resolving a {@link PhysicsEngineType} to an actual
 * {@link PhysicsEngineFactory} is done in {@code scs2-physics-engines}'s
 * {@code PhysicsEngineFactories}, the one module allowed to reference all four implementations at
 * compile time.
 * </p>
 * <p>
 * {@code DO_NOTHING} is deliberately not a member of this enum -- {@link DoNothingPhysicsEngine}
 * and {@link PhysicsEngineFactory#newDoNothingPhysicsEngineFactory()} stay reachable directly from
 * core so a consumer can get a fully working {@link us.ihmc.scs2.simulation.SimulationSession}
 * without ever depending on {@code scs2-physics-engines}.
 * </p>
 */
public enum PhysicsEngineType
{
   CONTACT_POINT_BASED,
   IMPULSE_BASED,
   BULLET,
   MUJOCO
}
