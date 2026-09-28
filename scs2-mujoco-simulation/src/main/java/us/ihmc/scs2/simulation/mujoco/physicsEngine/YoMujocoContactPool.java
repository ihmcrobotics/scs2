package us.ihmc.scs2.simulation.mujoco.physicsEngine;

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
 * Fixed pool of {@link YoMujocoContact} slots, refreshed from {@code mjData.contact} after every
 * step into a read-only {@code MujocoContactPool} child registry. Detail is capped at the capacity;
 * {@code contactOverflowCount} reports how many contacts went unrecorded. Capacity is fixed at
 * construction because the variables must exist before the session buffer is set up.
 */
public class YoMujocoContactPool
{
   private final YoRegistry registry = new YoRegistry("MujocoContactPool");
   private final YoMujocoContact[] slots;
   private final YoInteger contactOverflowCount;

   private mjModel model;
   private mjData data;
   private DoublePointer forceScratch;

   public YoMujocoContactPool(int capacity, ReferenceFrame worldFrame, YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      slots = new YoMujocoContact[capacity];
      for (int i = 0; i < capacity; i++)
         slots[i] = new YoMujocoContact(i, worldFrame, registry);
      contactOverflowCount = new YoInteger("contactOverflowCount", "Number of contacts beyond the pool capacity this tick, whose detail is not recorded", registry);
   }

   /** Caches native handles; call once, right after the model has compiled. */
   public void bind(mjModel model, mjData data)
   {
      this.model = model;
      this.data = data;
      forceScratch = new DoublePointer(6);
   }

   /** Refreshes all slots from the step that just completed; physics thread only. */
   public void update()
   {
      if (data == null)
         return;

      int ncon = data.ncon();
      contactOverflowCount.set(Math.max(0, ncon - slots.length));

      // mjData.contact and the efc_* arrays live in the arena, whose layout can change between
      // steps — re-fetch the base pointers every tick instead of caching them in bind().
      mjContact contact = data.contact();
      IntPointer efcState = data.efc_state();
      DoublePointer efcKBIP = data.efc_KBIP();
      IntPointer geom_bodyid = model.geom_bodyid();
      int nefc = data.nefc();

      int recorded = Math.min(ncon, slots.length);
      for (int contactId = 0; contactId < recorded; contactId++)
      {
         contact.position(contactId);
         Mujoco.mj_contactForce(model, data, contactId, forceScratch);
         slots[contactId].update(geom_bodyid, contact, forceScratch, efcState, efcKBIP, nefc);
      }

      for (int slotIndex = ncon; slotIndex < slots.length; slotIndex++)
         slots[slotIndex].clear();
   }
}
