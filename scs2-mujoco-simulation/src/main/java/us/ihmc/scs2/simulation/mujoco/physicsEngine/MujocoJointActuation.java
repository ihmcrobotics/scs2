package us.ihmc.scs2.simulation.mujoco.physicsEngine;

/**
 * One 1-DoF joint's low-level command under the joint servo, plus the torque decomposition for it.
 *
 * <p>The command is what the hardware drives receive: a feedforward torque, position and velocity
 * setpoints, and the gains to close the loop with. The engine reads it off {@code ControllerOutput}
 * once per tick; {@link #setCommand} is for anything wanting to drive a joint directly. MuJoCo then
 * evaluates {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} on every physics step in between.
 *
 * <p>The decomposition is computed from the state MuJoCo evaluated the actuator at, captured when
 * the command was pushed, so the terms are what produced the force rather than a controller-rate
 * prediction of it. They sum to {@link #getAppliedTau()} except when the joint's
 * {@code actuatorfrcrange} clamps it, since the decomposition is of the command and not of the
 * clamped result. Plain fields rather than YoVariables: the feedforward and the total are already
 * in the buffer under other names.
 */
public class MujocoJointActuation
{
   private final String jointName;
   /** Index of this joint's servo actuator, or -1 when the joint has none. */
   private final int actuatorIndex;
   /**
    * Address of the actuator's first control in {@code mjData.ctrl}. Equal to {@link #actuatorIndex}
    * for a single-control actuator, and NOT equal once any actuator in the model owns several.
    */
   private final int controlAddress;
   /** True when the actuator is a {@code <dcmotor>}: three controls, and gains in {@code gainprm}. */
   private final boolean dcMotor;

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
      this(jointName, actuatorIndex, actuatorIndex, false);
   }

   public MujocoJointActuation(String jointName, int actuatorIndex, int controlAddress, boolean dcMotor)
   {
      this.jointName = jointName;
      this.actuatorIndex = actuatorIndex;
      this.controlAddress = controlAddress;
      this.dcMotor = dcMotor;
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

   /** Address of this joint's first control in {@code mjData.ctrl}. */
   public int getControlAddress()
   {
      return controlAddress;
   }

   /** True when this joint is driven by a {@code <dcmotor>} rather than the affine servo. */
   public boolean isDcMotor()
   {
      return dcMotor;
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
