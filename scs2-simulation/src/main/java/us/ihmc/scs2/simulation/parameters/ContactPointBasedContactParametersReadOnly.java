package us.ihmc.scs2.simulation.parameters;

import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;

/**
 * Read-only interface for accessing the contact parameters used by the
 * {@link ContactPointBasedPhysicsEngine}.
 * <p>
 * The contact point based engine models each ground contact point as a soft spring-damper contact.
 * The normal direction is the contact normal. The two remaining directions form the tangent plane at
 * the contact point.
 * </p>
 */
public interface ContactPointBasedContactParametersReadOnly
{
   /**
    * Returns the tangential spring coefficient used in the contact plane.
    * <p>
    * This coefficient scales the contact point displacement from touchdown projected onto the two
    * tangent directions.
    * </p>
    *
    * @return the tangential spring coefficient.
    */
   double getKxy();

   /**
    * Returns the tangential damping coefficient used in the contact plane.
    * <p>
    * This coefficient damps the contact point velocity projected onto the two tangent directions.
    * </p>
    *
    * @return the tangential damping coefficient.
    */
   double getBxy();

   /**
    * Returns the normal spring coefficient.
    * <p>
    * This coefficient scales the contact point displacement along the contact normal. The normal
    * spring response is stiffened as the displacement approaches {@link #getStiffeningLength()}.
    * </p>
    *
    * @return the normal spring coefficient.
    */
   double getKz();

   /**
    * Returns the normal damping coefficient.
    * <p>
    * This coefficient damps the contact point velocity projected onto the contact normal.
    * </p>
    *
    * @return the normal damping coefficient.
    */
   double getBz();

   /**
    * Returns the characteristic penetration length used to stiffen the normal spring response.
    * <p>
    * Smaller values make the normal spring stiffen more rapidly as the contact point moves farther
    * from its touchdown position along the contact normal.
    * </p>
    *
    * @return the normal-contact stiffening length.
    */
   double getStiffeningLength();

   /**
    * Returns the force ratio used to cap tangential force while a contact point is already slipping.
    * <p>
    * When slip handling is enabled, the tangential force is limited using this ratio multiplied by
    * the normal force magnitude.
    * </p>
    *
    * @return the tangential-to-normal force ratio used for slipping contacts.
    */
   double getAlphaSlip();

   /**
    * Returns the force ratio used to decide when a sticking contact point starts slipping.
    * <p>
    * When slip handling is enabled, a contact point starts slipping when the tangential-to-normal
    * force ratio exceeds this value.
    * </p>
    *
    * @return the tangential-to-normal force ratio used to start slipping.
    */
   double getAlphaStick();

   /**
    * Returns whether stick-slip limiting is enabled for tangential contact forces.
    *
    * @return {@code true} to enable stick-slip force limiting, {@code false} to use the spring-damper
    *         force directly.
    */
   boolean isSlipEnabled();
}
