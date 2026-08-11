package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.util.HashMap;
import java.util.Map;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjContact;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Fixed pool of {@link YoMujocoContact} slots plus per-body {@link MujocoBodyContactAggregate}
 * totals, refreshed from {@code mjData.contact} after every step into a read-only
 * {@code MujocoContactPool} child registry. Slot detail is capped at the capacity (see
 * {@code contactOverflowCount}); aggregates always cover all {@code ncon} contacts. Capacity is
 * fixed at construction because the variables must exist before the session buffer is set up.
 */
public class YoMujocoContactPool
{
   private final YoRegistry registry = new YoRegistry("MujocoContactPool");
   private final YoMujocoContact[] slots;
   private final YoInteger contactOverflowCount;
   private final Map<Integer, MujocoBodyContactAggregate> aggregatesByBodyId = new HashMap<>();

   private mjModel model;
   private mjData data;
   private DoublePointer forceScratch;

   public YoMujocoContactPool(int capacity, ReferenceFrame worldFrame, YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      slots = new YoMujocoContact[capacity];
      for (int i = 0; i < capacity; i++)
         slots[i] = new YoMujocoContact(i, worldFrame, registry);
      contactOverflowCount = new YoInteger("contactOverflowCount",
                                           "Number of contacts beyond the pool capacity this tick (aggregates still cover them; slot detail does not)",
                                           registry);
   }

   /** Registers a body's aggregate; called once per collidable body at compile time. */
   public void addBodyAggregate(int mujocoBodyId, MujocoBodyContactAggregate aggregate)
   {
      aggregatesByBodyId.put(mujocoBodyId, aggregate);
   }

   /** Caches native handles; call once, right after the model has compiled. */
   public void bind(mjModel model, mjData data)
   {
      this.model = model;
      this.data = data;
      forceScratch = new DoublePointer(6);
   }

   /** Refreshes all slots and aggregates from the step that just completed; physics thread only. */
   public void update()
   {
      if (data == null)
         return;

      int ncon = data.ncon();
      contactOverflowCount.set(Math.max(0, ncon - slots.length));

      for (MujocoBodyContactAggregate aggregate : aggregatesByBodyId.values())
         aggregate.clear();

      // mjData.contact and the efc_* arrays live in the arena, whose layout can change between
      // steps — re-fetch the base pointers every tick instead of caching them in bind().
      mjContact contact = data.contact();
      IntPointer efcState = data.efc_state();
      DoublePointer efcKBIP = data.efc_KBIP();
      IntPointer geomBodyId = model.geom_bodyid();
      int nefc = data.nefc();

      for (int contactId = 0; contactId < ncon; contactId++)
      {
         contact.position(contactId);
         Mujoco.mj_contactForce(model, data, contactId, forceScratch);

         if (contactId < slots.length)
            slots[contactId].update(geomBodyId, contact, forceScratch, efcState, efcKBIP, nefc);

         accumulateAggregates(geomBodyId, contact);
      }

      for (int slotIndex = ncon; slotIndex < slots.length; slotIndex++)
         slots[slotIndex].clear();
   }

   private void accumulateAggregates(IntPointer geomBodyId, mjContact contact)
   {
      int geomIdA = contact.geom(0);
      int geomIdB = contact.geom(1);
      if (geomIdA < 0 || geomIdB < 0)
         return; // flex contact; no geom-owned bodies to attribute.

      MujocoBodyContactAggregate aggregateA = aggregatesByBodyId.get(geomBodyId.get(geomIdA));
      MujocoBodyContactAggregate aggregateB = aggregatesByBodyId.get(geomBodyId.get(geomIdB));
      if (aggregateA == null && aggregateB == null)
         return;

      // mj_contactForce returns the force in the contact frame, whose axes are the rows of
      // mjContact.frame: [0..2] normal (pointing geom A -> geom B), [3..5] tangent1, [6..8]
      // tangent2. That force acts on geom B's body; geom A's body sees the opposite.
      double normalForce = forceScratch.get(0);
      double forceOnBX = forceScratch.get(0) * contact.frame(0) + forceScratch.get(1) * contact.frame(3) + forceScratch.get(2) * contact.frame(6);
      double forceOnBY = forceScratch.get(0) * contact.frame(1) + forceScratch.get(1) * contact.frame(4) + forceScratch.get(2) * contact.frame(7);
      double forceOnBZ = forceScratch.get(0) * contact.frame(2) + forceScratch.get(1) * contact.frame(5) + forceScratch.get(2) * contact.frame(8);

      if (aggregateA != null)
         aggregateA.accumulate(normalForce, -forceOnBX, -forceOnBY, -forceOnBZ);
      if (aggregateB != null)
         aggregateB.accumulate(normalForce, forceOnBX, forceOnBY, forceOnBZ);
   }
}
