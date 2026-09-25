package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoActuationMode;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * One 1-DoF joint's low-level command under {@link MujocoActuationMode#JOINT_SERVO}, plus the
 * torque decomposition MuJoCo reports back for it.
 *
 * <p>The command is what the hardware drives receive: a feedforward torque, a position and
 * velocity setpoint, and the gains to close the loop with. Whoever bridges the controller to the
 * engine calls {@link #setCommand} once per controller tick; MuJoCo then evaluates
 * {@code tau_ff + kp * (q_d - q) + kd * (qd_d - qd)} on every physics step in between.
 *
 * <p>The three YoDoubles carry the names {@code SCS2OutputWriter} uses for the same quantities
 * ({@code <joint>LowLevelControllerTau} and friends) so existing plots and log comparisons keep
 * working. They differ in one respect worth knowing: these are the forces MuJoCo actually applied,
 * sampled at the physics rate and after the joint's {@code actuatorfrcrange} clamp, rather than
 * the controller-rate prediction the output writer computes.
 */
public class MujocoJointActuation
{
   private final String jointName;
   /** Index of this joint's first actuator; the other two follow it. -1 when the joint has none. */
   private final int actuatorBaseIndex;

   private double feedforwardTorque = 0.0;
   private double desiredPosition = 0.0;
   private double desiredVelocity = 0.0;
   private double stiffness = 0.0;
   private double damping = 0.0;

   // Gains live in mjModel rather than mjData, so they are only written when they actually change.
   private double writtenStiffness = Double.NaN;
   private double writtenDamping = Double.NaN;

   private final YoDouble yoControllerTau;
   private final YoDouble yoPositionTau;
   private final YoDouble yoVelocityTau;

   public MujocoJointActuation(String jointName, int actuatorBaseIndex, YoRegistry registry)
   {
      this.jointName = jointName;
      this.actuatorBaseIndex = actuatorBaseIndex;

      String prefix = jointName + "LowLevel";
      yoControllerTau = new YoDouble(prefix + "ControllerTau", registry);
      yoPositionTau = new YoDouble(prefix + "PositionTau", registry);
      yoVelocityTau = new YoDouble(prefix + "VelocityTau", registry);
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

   public int getActuatorBaseIndex()
   {
      return actuatorBaseIndex;
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

   void setRealizedTorques(double controllerTau, double positionTau, double velocityTau)
   {
      yoControllerTau.set(controllerTau);
      yoPositionTau.set(positionTau);
      yoVelocityTau.set(velocityTau);
   }

   /** The feedforward torque MuJoCo applied, as reported by {@code mjData.actuator_force}. */
   public double getRealizedControllerTau()
   {
      return yoControllerTau.getValue();
   }

   /** The position-feedback torque MuJoCo applied. */
   public double getRealizedPositionTau()
   {
      return yoPositionTau.getValue();
   }

   /** The velocity-feedback torque MuJoCo applied. */
   public double getRealizedVelocityTau()
   {
      return yoVelocityTau.getValue();
   }
}
