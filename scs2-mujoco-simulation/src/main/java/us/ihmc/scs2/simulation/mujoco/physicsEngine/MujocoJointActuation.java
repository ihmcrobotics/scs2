package us.ihmc.scs2.simulation.mujoco.physicsEngine;

/**
 * One 1-DoF joint's low-level command under the joint servo, plus the
 * torque decomposition for it.
 *
 * <p>The command is what the hardware drives receive: a feedforward torque, a position and
 * velocity setpoint, and the gains to close the loop with. The engine reads it off
 * {@code ControllerOutput} once per tick, where a controller publishes it alongside the effort;
 * {@link #setCommand} is for anything that wants to drive a joint directly instead. MuJoCo then
 * evaluates {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} on every physics step in between.
 *
 * <p>The torque decomposition is kept as plain fields rather than YoVariables. The feedforward term
 * is by construction the same number {@code SCS2OutputWriter} already publishes as
 * {@code <joint>LowLevelControllerTau}, and the applied total is what
 * {@code pullActuationFromMujoco} writes onto the joint as {@code tau_<joint>}, so putting either in
 * the buffer duplicates a variable that is already there. The two feedback terms are genuinely
 * MuJoCo's own -- evaluated at the state the actuator saw rather than at the controller's -- but the
 * buffer only samples once per recorded tick, so what it would capture is one arbitrary physics step
 * out of the ten or twenty in a controller tick, which is not the intra-tick detail that would make
 * them worth the variables. They stay available through the getters for tests and debugging.
 *
 * <p>The terms are computed from the state MuJoCo evaluated the actuator at -- the joint's q and qd
 * at the top of the tick, captured when the command was pushed -- so they are what actually produced
 * the force, not a controller-rate prediction of it. {@link #getAppliedTau()} carries the force
 * MuJoCo reports, and the three sum to it; a test pins that down rather than leaving the arithmetic
 * trusted.
 *
 * <p>The sum parts company with the applied force only when the joint's {@code actuatorfrcrange}
 * clamps it, since the decomposition is of the command rather than of the clamped result. That was
 * equally true when each term had its own actuator: MuJoCo clamps the joint's total, so the
 * per-actuator forces did not sum to it either.
 */
public class MujocoJointActuation
{
   private final String jointName;
   /** Index of this joint's servo actuator, or -1 when the joint has none. */
   private final int actuatorIndex;

   private double feedforwardTorque = 0.0;
   private double desiredPosition = 0.0;
   private double desiredVelocity = 0.0;
   private double stiffness = 0.0;
   private double damping = 0.0;

   // Gains live in mjModel rather than mjData, so they are only written when they actually change.
   private double writtenStiffness = Double.NaN;
   private double writtenDamping = Double.NaN;
   /** Cleared every tick: distinguishes "commanded zero" from "nobody commanded anything". */
   private boolean commandedThisTick = false;

   // The joint state MuJoCo evaluated the actuator at, captured when the command was pushed.
   private double positionAtCommand = 0.0;
   private double velocityAtCommand = 0.0;

   private double controllerTau;
   private double positionTau;
   private double velocityTau;
   private double appliedTau;

   public MujocoJointActuation(String jointName, int actuatorIndex)
   {
      this.jointName = jointName;
      this.actuatorIndex = actuatorIndex;
   }

   /**
    * Set the whole low-level command for this joint, in place of publishing it through
    * {@code ControllerOutput}. A term the controller did not provide should be passed as zero, which
    * makes it contribute nothing. A command published through {@code ControllerOutput} wins over one
    * staged here.
    */
   public void setCommand(double feedforwardTorque, double desiredPosition, double desiredVelocity, double stiffness, double damping)
   {
      set(feedforwardTorque, desiredPosition, desiredVelocity, stiffness, damping);
      commandedThisTick = true;
   }

   /** As {@link #setCommand}, but does not raise the staging flag: the engine is the caller. */
   void setCommandFromControllerOutput(double feedforwardTorque,
                                       double desiredPosition,
                                       double desiredVelocity,
                                       double stiffness,
                                       double damping)
   {
      set(feedforwardTorque, desiredPosition, desiredVelocity, stiffness, damping);
   }

   private void set(double feedforwardTorque, double desiredPosition, double desiredVelocity, double stiffness, double damping)
   {
      this.feedforwardTorque = feedforwardTorque;
      this.desiredPosition = desiredPosition;
      this.desiredVelocity = desiredVelocity;
      this.stiffness = stiffness;
      this.damping = damping;
   }

   /**
    * Fall back to SCS2's effort contract: drive the joint with the controller's torque alone. Same
    * force as {@code setCommand(effort, 0, 0, 0, 0)}, and the engine applies it when no command was
    * issued this tick.
    */
   void setFeedforwardOnly(double effort)
   {
      set(effort, 0.0, 0.0, 0.0, 0.0);
   }

   /** Consume-and-clear the per-tick command flag. */
   boolean pollCommanded()
   {
      boolean commanded = commandedThisTick;
      commandedThisTick = false;
      return commanded;
   }

   /** Zero the command, which leaves the joint free apart from its passive damping and friction. */
   public void clear()
   {
      setCommand(0.0, 0.0, 0.0, 0.0, 0.0);
   }

   public String getJointName()
   {
      return jointName;
   }

   public int getActuatorIndex()
   {
      return actuatorIndex;
   }

   /**
    * The {@code ctrl} value that makes a single affine actuator produce
    * {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qdot)}, given {@code biasprm = 0, -kp, -kd}.
    */
   double getCombinedControl()
   {
      return feedforwardTorque + stiffness * desiredPosition + damping * desiredVelocity;
   }

   /** Records the joint state MuJoCo will evaluate the actuator at, so the split can be exact. */
   void setStateAtCommand(double position, double velocity)
   {
      positionAtCommand = position;
      velocityAtCommand = velocity;
   }

   public double getFeedforwardTorque()
   {
      return feedforwardTorque;
   }

   public double getDesiredPosition()
   {
      return desiredPosition;
   }

   public double getDesiredVelocity()
   {
      return desiredVelocity;
   }

   public double getStiffness()
   {
      return stiffness;
   }

   public double getDamping()
   {
      return damping;
   }

   /** True when the gains differ from what was last written into {@code mjModel}. */
   boolean gainsNeedWriting()
   {
      return stiffness != writtenStiffness || damping != writtenDamping;
   }

   void markGainsWritten()
   {
      writtenStiffness = stiffness;
      writtenDamping = damping;
   }

   /**
    * Splits the command into its three terms using the state the actuator was evaluated at, and
    * records the force MuJoCo reported for comparison.
    */
   void updateTorqueDecomposition(double appliedTau)
   {
      controllerTau = feedforwardTorque;
      positionTau = stiffness * (desiredPosition - positionAtCommand);
      velocityTau = damping * (desiredVelocity - velocityAtCommand);
      this.appliedTau = appliedTau;
   }

   /** The feedforward term of the applied torque. */
   public double getControllerTau()
   {
      return controllerTau;
   }

   /** The position-feedback term of the applied torque. */
   public double getPositionTau()
   {
      return positionTau;
   }

   /** The velocity-feedback term of the applied torque. */
   public double getVelocityTau()
   {
      return velocityTau;
   }

   /** The force MuJoCo reported for this joint's actuator, which the three terms sum to. */
   public double getAppliedTau()
   {
      return appliedTau;
   }
}
