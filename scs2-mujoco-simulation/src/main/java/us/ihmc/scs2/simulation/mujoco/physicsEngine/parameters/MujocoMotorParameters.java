package us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters;

/**
 * One joint's drive described as a DC motor, for MuJoCo's {@code <dcmotor>} actuator.
 *
 * <p>Every value is JOINT side, so the emitted actuator carries {@code gear="1"}. The motor
 * equations keep their form under reflection, so with {@code Kj = Kt * N} a joint-side
 * {@code (Kj, R, maxTorque)} is a complete description and {@code Kt} and {@code N} never have to
 * be separated. Below its limits a {@code <dcmotor>} delivers the commanded torque exactly, as a
 * real current loop does; what it adds over the affine servo is the torque-speed envelope.
 *
 * @param torqueConstant  joint-side motor constant Kj [N*m/A], equivalently [V*s/rad]; torque per
 *                        amp and back-EMF per rad/s are the same number.
 * @param resistance      winding resistance R [ohm]. For a wye-wound brushless motor this is the
 *                        PHASE resistance, i.e. half the line-to-line value a datasheet quotes.
 * @param supplyVoltage   bus voltage [V]; sets the no-load speed {@code supplyVoltage / Kj}.
 * @param maxTorque       joint-side torque limit [N*m]. Pass 0 to inherit the joint's URDF effort limit.
 * @param inductance      winding inductance L [H], adding a current state of time constant
 *                        {@code L / R}. 0 keeps the quasi-static model.
 * @param viscousDamping  viscous loss [N*m*s/rad] applied after the current clamp; 0 to omit. Dry
 *                        friction and rotor inertia deliberately do NOT belong here: scs2 emits
 *                        them from the joint definition as {@code frictionloss} and
 *                        {@code armature}, and MuJoCo ADDS {@code actuator_armature} to
 *                        {@code dof_armature}, so a second copy would silently double the inertia.
 */
public record MujocoMotorParameters(double torqueConstant,
                                    double resistance,
                                    double supplyVoltage,
                                    double maxTorque,
                                    double inductance,
                                    double viscousDamping)
{
   public MujocoMotorParameters
   {
      requirePositive("torqueConstant", torqueConstant);
      requirePositive("resistance", resistance);
      requirePositive("supplyVoltage", supplyVoltage);
      requireNonNegative("maxTorque", maxTorque);
      requireNonNegative("inductance", inductance);
      requireNonNegative("viscousDamping", viscousDamping);
   }

   /** Motor constant, resistance, bus voltage and torque limit, with no electrical or loss terms. */
   public static MujocoMotorParameters quasiStatic(double torqueConstant,
                                                  double resistance,
                                                  double supplyVoltage,
                                                  double maxTorque)
   {
      return new MujocoMotorParameters(torqueConstant, resistance, supplyVoltage, maxTorque, 0.0, 0.0);
   }

   /**
    * Build from a no-load speed instead of a motor constant, via {@code Kj = supplyVoltage / noLoadSpeed}.
    *
    * <p><b>This assumes the quoted speed is the motor's unloaded electrical limit</b>, not a limit
    * imposed by a controller, a mechanical stop or a safety margin. A URDF {@code velocity} limit is
    * often the former and sometimes the latter, so check which one you have: if the quoted speed is
    * a derated figure, this understates Kj and the model will be stronger at speed than the real
    * drive, which is the wrong direction to be wrong in.
    */
   public static MujocoMotorParameters fromNoLoadSpeed(double supplyVoltage,
                                                       double noLoadSpeed,
                                                       double resistance,
                                                       double maxTorque)
   {
      requirePositive("noLoadSpeed", noLoadSpeed);
      requirePositive("supplyVoltage", supplyVoltage);
      return quasiStatic(supplyVoltage / noLoadSpeed, resistance, supplyVoltage, maxTorque);
   }

   /** Stall torque {@code Kj * supplyVoltage / R} [N*m]: the envelope's torque axis intercept. */
   public double stallTorque()
   {
      return torqueConstant * supplyVoltage / resistance;
   }

   /** No-load speed {@code supplyVoltage / Kj} [rad/s]: the envelope's speed axis intercept. */
   public double noLoadSpeed()
   {
      return supplyVoltage / torqueConstant;
   }

   /**
    * Torque available at {@code jointVelocity} [N*m], the lesser of the voltage-limited electrical
    * torque and {@link #maxTorque()}. Returns 0 past the no-load speed. With {@code maxTorque} left
    * at 0 the current clamp is not known here, so only the electrical limit is applied.
    */
   public double availableTorque(double jointVelocity)
   {
      double electrical = (torqueConstant / resistance) * (supplyVoltage - torqueConstant * Math.abs(jointVelocity));
      if (electrical < 0.0)
         return 0.0;
      return maxTorque > 0.0 ? Math.min(maxTorque, electrical) : electrical;
   }

   private static void requirePositive(String name, double value)
   {
      if (!(value > 0.0) || !Double.isFinite(value))
         throw new IllegalArgumentException("MujocoMotorParameters." + name + " must be finite and positive; got " + value);
   }

   private static void requireNonNegative(String name, double value)
   {
      if (!(value >= 0.0) || !Double.isFinite(value))
         throw new IllegalArgumentException("MujocoMotorParameters." + name + " must be finite and non-negative; got " + value);
   }
}
