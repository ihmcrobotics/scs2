package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Per-body contact totals (e.g. the normal force under a foot), re-accumulated each step by
 * {@link YoMujocoContactPool} into the owning robot's registry, prefixed with the body name.
 */
public class MujocoBodyContactAggregate
{
   private final YoInteger numberOfContacts;
   private final YoDouble totalContactNormalForce;
   private final YoFrameVector3D totalContactForce;

   public MujocoBodyContactAggregate(String bodyName, ReferenceFrame worldFrame, YoRegistry registry)
   {
      numberOfContacts = new YoInteger(bodyName + "NumberOfContacts", "Number of active MuJoCo contacts involving " + bodyName, registry);
      totalContactNormalForce = new YoDouble(bodyName + "TotalContactNormalForce",
                                             "Sum of contact normal force magnitudes [N] on " + bodyName,
                                             registry);
      totalContactForce = new YoFrameVector3D(bodyName + "TotalContactForce", worldFrame, registry);
   }

   public void clear()
   {
      numberOfContacts.set(0);
      totalContactNormalForce.set(0.0);
      totalContactForce.setToZero();
   }

   /** The force components are in world coordinates, sign-corrected for which side of the contact this body is on. */
   public void accumulate(double normalForceMagnitude, double forceX, double forceY, double forceZ)
   {
      numberOfContacts.increment();
      totalContactNormalForce.add(normalForceMagnitude);
      totalContactForce.add(forceX, forceY, forceZ);
   }
}
