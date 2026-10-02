package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjContact;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicArrow3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint3D;

import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * One pre-allocated per-contact YoVariable slot ({@code dist_0}, {@code normalForce_3}, ...),
 * filled from {@code mjData.contact[i]} + {@code mj_contactForce} by {@link YoMujocoContactPool}.
 *
 * <p>A slot follows one physical contact for as long as that contact exists. MuJoCo's own contact
 * ordering is an arena index that is free to permute between steps, so indexing slots by it makes
 * two slots swap the corner they describe mid-stance -- which is worse than useless when the whole
 * point is to watch one corner of a foot through a touchdown. {@link YoMujocoContactPool} matches
 * each contact back to its slot on the geom pair plus proximity instead; {@code age} counts the
 * ticks a slot has held the same contact, so a slot that keeps resetting is chatter.
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
   /** This slot's index in the pool; kept so the yoGraphics can be named after it. */
   private final int index;
   private final YoFrameVector3D normal;
   /**
    * {@code normal} scaled by {@code normalForce}, in newtons. Exists so a yoGraphic arrow can show
    * how hard the contact is pushing rather than only which way; nothing else reads it.
    */
   private final YoFrameVector3D normalForceVector;
   private final YoInteger geom_a;
   private final YoInteger geom_b;
   private final YoInteger body_a;
   private final YoInteger body_b;
   private final YoInteger age;

   // Plain fields, read every tick by the pool's matching pass before any YoVariable is written.
   private boolean occupied = false;
   private int matchGeomA = -1, matchGeomB = -1;
   private double matchX, matchY, matchZ;

   public YoMujocoContact(int index, ReferenceFrame worldFrame, YoRegistry registry)
   {
      this.index = index;
      geom_a = new YoInteger("geom_a_" + index, "mjContact.geom[0]: id of the first geom", registry);
      geom_b = new YoInteger("geom_b_" + index, "mjContact.geom[1]: id of the second geom", registry);
      body_a = new YoInteger("body_a_" + index, "MuJoCo body id owning geom A", registry);
      body_b = new YoInteger("body_b_" + index, "MuJoCo body id owning geom B", registry);
      dist = new YoDouble("dist_" + index, "mjContact.dist: distance between nearest points; negative = penetration depth [m]", registry);
      dim = new YoInteger("dim_" + index, "mjContact.dim: contact space dimensionality (1, 3, 4 or 6)", registry);
      pos = new YoFramePoint3D("pos_" + index, worldFrame, registry);
      normal = new YoFrameVector3D("normal_" + index, worldFrame, registry);
      normalForceVector = new YoFrameVector3D("normalForceVector_" + index, worldFrame, registry);
      normalForce = new YoDouble("normalForce_" + index, "mj_contactForce[0]: contact normal force [N], always >= 0", registry);
      tangentialForce = new YoDouble("tangentialForce_" + index, "Norm of mj_contactForce[1..2]: tangential friction force magnitude [N]", registry);
      slipping = new YoBoolean("slipping_" + index, "True when efc_state at this contact is on the friction-cone boundary (LINEARNEG/LINEARPOS/CONE)", registry);
      age = new YoInteger("age_" + index, "Consecutive ticks this slot has tracked the same physical contact; 0 means the slot is free", registry);
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

      // The A/B order is NOT physical: MuJoCo sorts each pair to match its collision-function table,
      // which is keyed by geom type with the lower mjGEOM_* index first. A box foot against a box
      // terrain keeps terrain as A (equal types, ordered by id) so the normal points up at the robot;
      // swap the foot to a SPHERE (type 2 < BOX 6) and the foot becomes A, so the same normal points
      // DOWN into the ground. The direction therefore flips with foot geometry alone.
      //
      // For display, always show the force acting on the non-world body, which is the reaction
      // pushing the robot. MuJoCo body 0 is always the world, so negate when the world is B. For a
      // robot self-collision neither body is the world and the arrow shows the force on body B.
      normalForceVector.set(normal);
      normalForceVector.scale(body_b.getValue() == 0 ? -normalForce.getValue() : normalForce.getValue());

      occupied = true;
      matchGeomA = geomIdA;
      matchGeomB = geomIdB;
      matchX = contact.pos(0);
      matchY = contact.pos(1);
      matchZ = contact.pos(2);
      age.increment();

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

   /** True when this slot held a contact on the previous tick, so it is a candidate for matching. */
   boolean isOccupied()
   {
      return occupied;
   }

   /** True when the slot's previous contact was between the same pair of geoms, in either order. */
   boolean matchesGeomPair(int geomA, int geomB)
   {
      return (matchGeomA == geomA && matchGeomB == geomB) || (matchGeomA == geomB && matchGeomB == geomA);
   }

   /** Squared distance from this slot's previous contact point, for nearest-match against a geom pair. */
   double distanceSquaredFromPrevious(double x, double y, double z)
   {
      double dx = x - matchX, dy = y - matchY, dz = z - matchZ;
      return dx * dx + dy * dy + dz * dz;
   }

   /**
    * A sphere at the contact point and an arrow along the normal scaled by the normal force, so the
    * 3D view shows which contacts exist this tick and how hard each is pushing. A free slot holds NaN
    * and is simply not drawn, so the group needs no show/hide logic.
    * <p>
    * Colour is fixed per graphic -- {@link PaintDefinition} is not data-driven -- so {@code slipping}
    * cannot tint the sphere. Read it from the variable instead; the arrow's length carries the load.
    */
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition("Contact" + index);
      group.addChild(newYoGraphicPoint3D("contactPoint" + index, pos, 0.012, ColorDefinitions.Crimson()));
      // 0.002 m per newton: a 500 N foot contact draws a 1 m arrow, legible next to a standing human.
      group.addChild(newYoGraphicArrow3D("contactForce" + index, pos, normalForceVector, 0.002, ColorDefinitions.Gold()));
      return group;
   }

   public void clear()
   {
      occupied = false;
      matchGeomA = -1;
      matchGeomB = -1;
      age.set(0);

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
      // NaN rather than zero: a yoGraphic at a NaN position is not drawn, which is how a free slot
      // disappears from the 3D view instead of piling up at the origin.
      normalForceVector.setToNaN();
   }
}
