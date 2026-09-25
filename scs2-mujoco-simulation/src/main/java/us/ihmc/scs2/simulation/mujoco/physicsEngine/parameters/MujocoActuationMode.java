package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * How controller commands reach the MuJoCo joints.
 *
 * <p>Compile-time, because it decides whether the generated MJCF carries an {@code <actuator>}
 * block at all.
 */
public enum MujocoActuationMode
{
   /**
    * The engine writes the joint effort SCS2 already computed straight into
    * {@code mjData.qfrc_applied} as a generalized force. No actuators are emitted.
    *
    * <p>This is what every SCS2 engine has always done, and it is the default. The whole low-level
    * control law runs on the controller side, at the controller's rate, and the resulting torque is
    * held constant across the physics steps in between.
    */
   TORQUE_PASSTHROUGH,
   /**
    * The engine emits three actuators per 1-DoF joint and hands MuJoCo the setpoints and gains
    * instead of a torque, so MuJoCo evaluates
    * {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} itself on every physics step.
    *
    * <p>This mirrors the hardware signal path, where the drive closes the impedance loop at its own
    * rate from a setpoint the controller updates more slowly. It also lets MuJoCo integrate the
    * damping term implicitly (with the {@code IMPLICITFAST} integrator SCS2 defaults to), which an
    * explicit torque can never be at any rate, and lets MuJoCo clamp the total through the joint's
    * {@code actuatorfrcrange} and report each term separately through {@code actuator_force}.
    */
   JOINT_SERVO
}
