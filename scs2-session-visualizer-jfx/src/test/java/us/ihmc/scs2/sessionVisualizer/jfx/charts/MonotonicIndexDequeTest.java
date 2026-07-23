package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class MonotonicIndexDequeTest
{
   @Test
   public void testMonotonicIncreasingSequenceCollapsesToSizeOne()
   {
      double[] values = {1, 2, 3, 4, 5};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, true);

      for (int i = 0; i < values.length; i++)
      {
         deque.pushCandidate(i, values[i], values);
         assertEquals(1, deque.size());
         assertEquals(i, deque.peekFrontIndex());
      }
   }

   @Test
   public void testMonotonicDecreasingSequenceGrowsByOneEachPush()
   {
      double[] values = {5, 4, 3, 2, 1};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, true);

      for (int i = 0; i < values.length; i++)
      {
         deque.pushCandidate(i, values[i], values);
         assertEquals(i + 1, deque.size());
         assertEquals(0, deque.peekFrontIndex()); // first (largest) value never dominated
      }
   }

   @Test
   public void testMonotonicIncreasingSequenceMinTracking()
   {
      double[] values = {1, 2, 3, 4, 5};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, false);

      for (int i = 0; i < values.length; i++)
      {
         deque.pushCandidate(i, values[i], values);
         assertEquals(i + 1, deque.size());
         assertEquals(0, deque.peekFrontIndex()); // first (smallest) value never dominated
      }
   }

   @Test
   public void testMonotonicDecreasingSequenceMinTrackingCollapsesToSizeOne()
   {
      double[] values = {5, 4, 3, 2, 1};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, false);

      for (int i = 0; i < values.length; i++)
      {
         deque.pushCandidate(i, values[i], values);
         assertEquals(1, deque.size());
         assertEquals(i, deque.peekFrontIndex());
      }
   }

   @Test
   public void testTiesCollapseToNewestDuplicate()
   {
      double[] values = {0, 0, 0, 0, 0};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, true);

      for (int i = 0; i < values.length; i++)
      {
         deque.pushCandidate(i, values[i], values);
         assertEquals(1, deque.size());
         assertEquals(i, deque.peekFrontIndex());
      }

      // Evicting any of the older duplicates should be a no-op: they were already popped.
      for (int i = 0; i < values.length - 1; i++)
      {
         deque.evictIfFront(i);
         assertEquals(1, deque.size());
         assertEquals(values.length - 1, deque.peekFrontIndex());
      }
   }

   @Test
   public void testEvictionOfCurrentFrontAdvancesToNextSurvivor()
   {
      // Strictly decreasing so every entry survives, in push order: 0(5),1(4),2(3).
      double[] values = {5, 4, 3};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, true);
      for (int i = 0; i < values.length; i++)
         deque.pushCandidate(i, values[i], values);

      assertEquals(3, deque.size());
      assertEquals(0, deque.peekFrontIndex());

      deque.evictIfFront(0);
      assertEquals(2, deque.size());
      assertEquals(1, deque.peekFrontIndex());

      deque.evictIfFront(1);
      assertEquals(1, deque.size());
      assertEquals(2, deque.peekFrontIndex());

      deque.evictIfFront(2);
      assertEquals(0, deque.size());
      assertTrue(deque.isEmpty());
   }

   @Test
   public void testEvictionOfNonFrontDominatedIndexIsNoOp()
   {
      double[] values = {1, 5}; // index 0 (value 1) gets dominated/popped when index 1 (value 5) arrives.
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, true);
      deque.pushCandidate(0, values[0], values);
      deque.pushCandidate(1, values[1], values);

      assertEquals(1, deque.size());
      assertEquals(1, deque.peekFrontIndex());

      deque.evictIfFront(0); // already dominated, should be a no-op
      assertEquals(1, deque.size());
      assertEquals(1, deque.peekFrontIndex());
   }

   @Test
   public void testRingIndexWraparoundIsHandledCorrectly()
   {
      double[] values = new double[8];
      values[6] = 3;
      values[7] = 5;
      values[0] = 2;
      values[1] = 4;
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, true);

      deque.pushCandidate(6, values[6], values); // 3
      deque.pushCandidate(7, values[7], values); // 5 dominates 3
      assertEquals(1, deque.size());
      assertEquals(7, deque.peekFrontIndex());

      deque.pushCandidate(0, values[0], values); // 2, does not dominate 5
      assertEquals(2, deque.size());
      assertEquals(7, deque.peekFrontIndex());

      deque.pushCandidate(1, values[1], values); // 4, dominates 2 but not 5
      assertEquals(2, deque.size());
      assertEquals(7, deque.peekFrontIndex());

      deque.evictIfFront(7);
      assertEquals(1, deque.size());
      assertEquals(1, deque.peekFrontIndex());
   }

   @Test
   public void testInterleavedPushAndEvictMatchesBruteForce()
   {
      // Simulates a real ring buffer: the active window can never hold more than bufferSize indices, so
      // eviction is forced whenever an insert would otherwise exceed capacity -- exactly the invariant
      // incrementallyPatchValuesAndBounds relies on (evict aged-out indices, then insert new ones).
      Random random = new Random(1234L);
      int bufferSize = 16;
      double[] values = new double[bufferSize];
      MonotonicIndexDeque maxDeque = new MonotonicIndexDeque(bufferSize, true);
      MonotonicIndexDeque minDeque = new MonotonicIndexDeque(bufferSize, false);

      java.util.LinkedList<Integer> activeIndices = new java.util.LinkedList<>();

      int nextIndex = 0;
      for (int tick = 0; tick < 500; tick++)
      {
         int insertCount = 1 + random.nextInt(3);

         // Evict just enough (oldest-first) to make room, plus occasionally a little extra, but never all.
         int mandatoryEvictions = Math.max(0, activeIndices.size() + insertCount - bufferSize);
         int extraEvictions = activeIndices.size() > mandatoryEvictions ? random.nextInt(2) : 0;
         int evictCount = Math.min(mandatoryEvictions + extraEvictions, Math.max(0, activeIndices.size() - 1));
         evictCount = Math.max(evictCount, mandatoryEvictions);
         for (int i = 0; i < evictCount; i++)
         {
            int evicted = activeIndices.removeFirst();
            maxDeque.evictIfFront(evicted);
            minDeque.evictIfFront(evicted);
         }

         for (int i = 0; i < insertCount; i++)
         {
            int index = nextIndex % bufferSize;
            double value = random.nextDouble() * 20.0 - 10.0;
            values[index] = value;
            maxDeque.pushCandidate(index, value, values);
            minDeque.pushCandidate(index, value, values);
            activeIndices.addLast(index);
            nextIndex++;
         }

         double bruteMax = Double.NEGATIVE_INFINITY;
         double bruteMin = Double.POSITIVE_INFINITY;
         for (int index : activeIndices)
         {
            bruteMax = Math.max(bruteMax, values[index]);
            bruteMin = Math.min(bruteMin, values[index]);
         }

         assertEquals(bruteMax, values[maxDeque.peekFrontIndex()], 0.0, "tick " + tick);
         assertEquals(bruteMin, values[minDeque.peekFrontIndex()], 0.0, "tick " + tick);
      }
   }

   @Test
   public void testClearResetsToEmpty()
   {
      double[] values = {1, 2, 3};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(values.length, true);
      deque.pushCandidate(0, values[0], values);
      deque.pushCandidate(1, values[1], values);
      assertFalse(deque.isEmpty());

      deque.clear();
      assertTrue(deque.isEmpty());
      assertEquals(0, deque.size());
   }

   @Test
   public void testCapacityExceededThrows()
   {
      // Strictly decreasing so nothing gets dominated/popped, filling the deque to capacity.
      double[] values = {2, 1, 0};
      MonotonicIndexDeque deque = new MonotonicIndexDeque(2, true);
      deque.pushCandidate(0, values[0], values);
      deque.pushCandidate(1, values[1], values);
      assertEquals(2, deque.size());

      assertThrows(IllegalStateException.class, () -> deque.pushCandidate(2, values[2], values));
   }
}
