package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.yoGraphic.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
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
public class YoMujocoContactPool implements SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry("MujocoContactPool");
   private final YoMujocoContact[] slots;
   private final YoInteger contactOverflowCount;
   /** Per-tick scratch for slot assignment; sized at construction, never reallocated. */
   private final boolean[] claimed;
   private final int[] assignment;
   /** How far a contact point may move between ticks and still be considered the same contact [m]. */
   private static final double MATCH_GATE = 0.02;

   private mjModel model;
   private mjData data;
   private DoublePointer forceScratch;

   public YoMujocoContactPool(int capacity, ReferenceFrame worldFrame, YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      slots = new YoMujocoContact[capacity];
      claimed = new boolean[capacity];
      assignment = new int[capacity];
      for (int i = 0; i < capacity; i++)
         slots[i] = new YoMujocoContact(i, worldFrame, registry);
      contactOverflowCount = new YoInteger("contactOverflowCount", "Number of contacts beyond the pool capacity this tick, whose detail is not recorded", registry);
   }

   /**
    * A sphere and a force arrow per slot, the graphical form of this pool: the 3D view then shows
    * exactly which contacts MuJoCo has this tick and how hard each is pushing. Free slots hold NaN
    * and are not drawn, so the group is correct at every tick with no bookkeeping.
    * <p>
    * Contacts beyond the pool capacity are not drawn either -- check {@code contactOverflowCount}
    * before concluding the view is complete.
    */
   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition("MujocoContacts");
      for (YoMujocoContact slot : slots)
         group.addChild(slot.getSCS2YoGraphics());
      return group;
   }

   /**
    * The slot holding this contact last tick: same geom pair, nearest previous position, and still
    * unclaimed. The gate is generous -- a contact point travels at most a couple of millimetres in a
    * 2 ms step -- but finite, so a genuinely new contact does not steal an existing slot's identity.
    */
   private int findMatchingSlot(int geomA, int geomB, double x, double y, double z)
   {
      int best = -1;
      double bestDistanceSquared = MATCH_GATE * MATCH_GATE;
      for (int i = 0; i < slots.length; i++)
      {
         if (claimed[i] || !slots[i].isOccupied() || !slots[i].matchesGeomPair(geomA, geomB))
            continue;
         double d2 = slots[i].distanceSquaredFromPrevious(x, y, z);
         if (d2 < bestDistanceSquared)
         {
            bestDistanceSquared = d2;
            best = i;
         }
      }
      return best;
   }

   private int firstUnclaimedSlot(boolean allowOccupied)
   {
      for (int i = 0; i < slots.length; i++)
         if (!claimed[i] && (allowOccupied || !slots[i].isOccupied()))
            return i;
      return -1;
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

      // Assign each contact to the slot that held it last tick, so a slot follows one physical
      // contact instead of whatever MuJoCo's arena ordering puts at that index this step.
      int recorded = Math.min(ncon, slots.length);
      java.util.Arrays.fill(claimed, false);
      java.util.Arrays.fill(assignment, -1);

      for (int contactId = 0; contactId < recorded; contactId++)
      {
         contact.position(contactId);
         assignment[contactId] = findMatchingSlot(contact.geom(0), contact.geom(1), contact.pos(0), contact.pos(1), contact.pos(2));
         if (assignment[contactId] >= 0)
            claimed[assignment[contactId]] = true;
      }

      // Anything unmatched -- a new contact, or one that moved further than the gate -- takes a
      // free slot, preferring slots that were empty over ones whose contact has just ended.
      for (int contactId = 0; contactId < recorded; contactId++)
      {
         if (assignment[contactId] >= 0)
            continue;
         int free = firstUnclaimedSlot(false);
         if (free < 0)
            free = firstUnclaimedSlot(true);
         if (free < 0)
            continue; // More simultaneous contacts than slots; contactOverflowCount reports it.
         assignment[contactId] = free;
         claimed[free] = true;
         slots[free].clear();
      }

      for (int contactId = 0; contactId < recorded; contactId++)
      {
         int slot = assignment[contactId];
         if (slot < 0)
            continue;
         contact.position(contactId);
         Mujoco.mj_contactForce(model, data, contactId, forceScratch);
         slots[slot].update(geom_bodyid, contact, forceScratch, efcState, efcKBIP, nefc);
      }

      for (int slotIndex = 0; slotIndex < slots.length; slotIndex++)
         if (!claimed[slotIndex])
            slots[slotIndex].clear();
   }
}
