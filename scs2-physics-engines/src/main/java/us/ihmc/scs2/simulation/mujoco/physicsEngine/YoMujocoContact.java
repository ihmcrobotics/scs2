package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjContact;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * One pre-allocated per-contact YoVariable slot ({@code dist_0}, {@code normalForce_3}, ...),
 * filled from {@code mjData.contact[i]} + {@code mj_contactForce} by {@link YoMujocoContactPool}.
 */
public class YoMujocoContact
{
   // ---------- SCS2-owned ----------
   private final YoDouble normalForce;
   private final YoDouble tangentialForce;
   private final YoBoolean slipping;
   private final YoDouble impedance;
   // ---------- MuJoCo-owned (mjContact) ----------
   private final YoDouble dist;
   private final YoInteger dim;
   private final YoFramePoint3D pos;
   private final YoFrameVector3D normal;
   private final YoInteger geom_a;
   private final YoInteger geom_b;
   private final YoInteger body_a;
   private final YoInteger body_b;

   public YoMujocoContact(int index, ReferenceFrame worldFrame, YoRegistry registry)
   {
      geom_a = new YoInteger("geom_a_" + index, "mjContact.geom[0]: id of the first geom", registry);
      geom_b = new YoInteger("geom_b_" + index, "mjContact.geom[1]: id of the second geom", registry);
      body_a = new YoInteger("body_a_" + index, "MuJoCo body id owning geom A", registry);
      body_b = new YoInteger("body_b_" + index, "MuJoCo body id owning geom B", registry);
      dist = new YoDouble("dist_" + index, "mjContact.dist: distance between nearest points; negative = penetration depth [m]", registry);
      dim = new YoInteger("dim_" + index, "mjContact.dim: contact space dimensionality (1, 3, 4 or 6)", registry);
      pos = new YoFramePoint3D("pos_" + index, worldFrame, registry);
      normal = new YoFrameVector3D("normal_" + index, worldFrame, registry);
      normalForce = new YoDouble("normalForce_" + index, "mj_contactForce[0]: contact normal force [N], always >= 0", registry);
      tangentialForce = new YoDouble("tangentialForce_" + index, "Norm of mj_contactForce[1..2]: tangential friction force magnitude [N]", registry);
      slipping = new YoBoolean("slipping_" + index, "True when efc_state at this contact is on the friction-cone boundary (LINEARNEG/LINEARPOS/CONE)", registry);
      impedance = new YoDouble("impedance_" + index, "efc_KBIP[4*efc_address+2]: realized constraint impedance d — where on the solimp sigmoid this contact sits", registry);
      clear();
   }

   /** {@code contact} is pre-positioned at this contact's index; {@code forceScratch} was filled by {@code mj_contactForce}. */
   public void update(IntPointer geom_bodyid, mjContact contact, DoublePointer forceScratch, IntPointer efcState, DoublePointer efcKBIP, int nefc)
   {
      normalForce.set(forceScratch.get(0));
      tangentialForce.set(Math.hypot(forceScratch.get(1), forceScratch.get(2)));

      int geomIdA = contact.geom(0);
      int geomIdB = contact.geom(1);
      geom_a.set(geomIdA);
      geom_b.set(geomIdB);
      body_a.set(geomIdA >= 0 ? geom_bodyid.get(geomIdA) : -1);
      body_b.set(geomIdB >= 0 ? geom_bodyid.get(geomIdB) : -1);
      dist.set(contact.dist());
      dim.set(contact.dim());
      pos.set(contact.pos(0), contact.pos(1), contact.pos(2));
      // frame[0..2] is the contact normal, pointing from geom A to geom B.
      normal.set(contact.frame(0), contact.frame(1), contact.frame(2));

      int efcAddress = contact.efc_address();
      if (efcAddress >= 0 && efcAddress < nefc)
      {
         int state = efcState.get(efcAddress);
         slipping.set(state == Mujoco.mjCNSTRSTATE_LINEARNEG || state == Mujoco.mjCNSTRSTATE_LINEARPOS || state == Mujoco.mjCNSTRSTATE_CONE);
         impedance.set(efcKBIP.get(4L * efcAddress + 2));
      }
      else
      {
         slipping.set(false);
         impedance.set(Double.NaN);
      }
   }

   public void clear()
   {
      normalForce.set(Double.NaN);
      tangentialForce.set(Double.NaN);
      slipping.set(false);
      impedance.set(Double.NaN);

      geom_a.set(-1);
      geom_b.set(-1);
      body_a.set(-1);
      body_b.set(-1);
      dist.set(Double.NaN);
      dim.set(-1);
      pos.setToNaN();
      normal.setToNaN();
   }
}
