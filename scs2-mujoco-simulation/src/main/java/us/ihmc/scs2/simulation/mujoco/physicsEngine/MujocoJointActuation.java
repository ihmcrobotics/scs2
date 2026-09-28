package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * One 1-DoF joint's low-level command under the joint servo, plus the
 * torque decomposition for it.
 *
 * <p>The command is what the hardware drives receive: a feedforward torque, a position and
 * velocity setpoint, and the gains to close the loop with. Whoever bridges the controller to the
 * engine calls {@link #setCommand} once per controller tick; MuJoCo then evaluates
 * {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} on every physics step in between.
 *
 * <p>The three YoDoubles carry the names {@code SCS2OutputWriter} uses for the same quantities
 * ({@code <joint>LowLevelControllerTau} and friends) so existing plots and log comparisons keep
 * working. They are computed from the state MuJoCo evaluated the actuator at -- the joint's q and
 * qd at the top of the tick, captured when the command was pushed -- so they are the terms that
 * actually produced the force, not a controller-rate prediction of them. {@link #getAppliedTau()}
 * carries the force MuJoCo reports, and the three sum to it; a test pins that down rather than
 * leaving the arithmetic trusted.
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

   private final YoDouble yoControllerTau;
   private final YoDouble yoPositionTau;
   private final YoDouble yoVelocityTau;
   private final YoDouble yoAppliedTau;

   public MujocoJointActuation(String jointName, int actuatorIndex, YoRegistry registry)
   {
      this.jointName = jointName;
      this.actuatorIndex = actuatorIndex;

      String prefix = jointName + "LowLevel";
      yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
      yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
      yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);
      yoAppliedTau = new YoDouble(prefix + "AppliedTau", "Force MuJoCo reports for this joint's servo actuator; the three terms above sum to it", registry);
   }

   /**
    * Set the whole low-level command for this joint. A term the controller did not provide should
    * be passed as zero, which makes it contribute nothing.
    */
   public void setCommand(double feedforwardTorque, double desiredPosition, double desiredVelocity, double stiffness, double damping)
   {
      this.feedforwardTorque = feedforwardTorque;
      this.desiredPosition = desiredPosition;
      this.desiredVelocity = desiredVelocity;
      this.stiffness = stiffness;
      this.damping = damping;
      commandedThisTick = true;
   }

   /**
    * Fall back to SCS2's effort contract: drive the joint with the controller's torque alone. Same
    * force as {@code setCommand(effort, 0, 0, 0, 0)}, and the engine applies it when no command was
    * issued this tick.
    */
   void setFeedforwardOnly(double effort)
   {
      feedforwardTorque = effort;
      desiredPosition = 0.0;
      desiredVelocity = 0.0;
      stiffness = 0.0;
      damping = 0.0;
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
      yoControllerTau.set(feedforwardTorque);
      yoPositionTau.set(stiffness * (desiredPosition - positionAtCommand));
      yoVelocityTau.set(damping * (desiredVelocity - velocityAtCommand));
      yoAppliedTau.set(appliedTau);
   }

   /** The feedforward term of the applied torque. */
   public double getControllerTau()
   {
      return yoControllerTau.getValue();
   }

   /** The position-feedback term of the applied torque. */
   public double getPositionTau()
   {
      return yoPositionTau.getValue();
   }

   /** The velocity-feedback term of the applied torque. */
   public double getVelocityTau()
   {
      return yoVelocityTau.getValue();
   }

   /** The force MuJoCo reported for this joint's actuator, which the three terms sum to. */
   public double getAppliedTau()
   {
      return yoAppliedTau.getValue();
   }
}
