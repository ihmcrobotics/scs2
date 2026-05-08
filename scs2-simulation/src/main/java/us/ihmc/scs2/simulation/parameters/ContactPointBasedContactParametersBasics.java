package us.ihmc.scs2.simulation.parameters;

import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;

/**
 * Write and read interface for configuring the contact parameters used by the
 * {@link ContactPointBasedPhysicsEngine}.
 */
public interface ContactPointBasedContactParametersBasics extends ContactPointBasedContactParametersReadOnly
{
   /**
    * Performs a deep copy of {@code other} into {@code this}.
    *
    * @param other the other set of contact parameters. Not modified.
    */
   default void set(ContactPointBasedContactParametersReadOnly other)
   {
      setKxy(other.getKxy());
      setBxy(other.getBxy());
      setKz(other.getKz());
      setBz(other.getBz());
      setStiffeningLength(other.getStiffeningLength());
      setAlphaSlip(other.getAlphaSlip());
      setAlphaStick(other.getAlphaStick());
      setEnableSlip(other.isSlipEnabled());
   }

   /**
    * Sets the tangential spring coefficient used in the contact plane.
    *
    * @param kxy the tangential spring coefficient.
    * @see #getKxy()
    */
   void setKxy(double kxy);

   /**
    * Sets the tangential damping coefficient used in the contact plane.
    *
    * @param bxy the tangential damping coefficient.
    * @see #getBxy()
    */
   void setBxy(double bxy);

   /**
    * Sets the normal spring coefficient.
    *
    * @param kz the normal spring coefficient.
    * @see #getKz()
    */
   void setKz(double kz);

   /**
    * Sets the normal damping coefficient.
    *
    * @param bz the normal damping coefficient.
    * @see #getBz()
    */
   void setBz(double bz);

   /**
    * Sets the characteristic penetration length used to stiffen the normal spring response.
    *
    * @param stiffeningLength the normal-contact stiffening length.
    * @see #getStiffeningLength()
    */
   void setStiffeningLength(double stiffeningLength);

   /**
    * Sets the tangential-to-normal force ratio used for slipping contacts.
    *
    * @param alphaSlip the force ratio used while slipping.
    * @see #getAlphaSlip()
    */
   void setAlphaSlip(double alphaSlip);

   /**
    * Sets the tangential-to-normal force ratio used to start slipping.
    *
    * @param alphaStick the force ratio used to start slipping.
    * @see #getAlphaStick()
    */
   void setAlphaStick(double alphaStick);

   /**
    * Enables or disables stick-slip limiting for tangential contact forces.
    *
    * @param enableSlip {@code true} to enable stick-slip force limiting, {@code false} to use the
    *                   spring-damper force directly.
    * @see #isSlipEnabled()
    */
   void setEnableSlip(boolean enableSlip);
}
