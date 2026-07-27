package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Deque of indices, maintained so that the entry at the front is always the current extremum
 * (max, or min, depending on {@code trackMax}) of the indices currently being held. Backed by an
 * {@link ArrayDeque}, so this is not off the JavaFX application thread's allocation-free hot path
 * -- boxing {@code Integer}s is an acceptable tradeoff here since this is not a realtime thread.
 */
class MonotonicIndexDeque
{
   private final Deque<Integer> indices = new ArrayDeque<>();
   private final int ringBufferSize;
   private final boolean trackMax;

   MonotonicIndexDeque(int ringBufferSize, boolean trackMax)
   {
      this.ringBufferSize = ringBufferSize;
      this.trackMax = trackMax;
   }

   void clear()
   {
      indices.clear();
   }

   boolean isEmpty()
   {
      return indices.isEmpty();
   }

   int size()
   {
      return indices.size();
   }

   /** Ring-buffer index of the current extremum candidate. Caller must check {@link #isEmpty()} first. */
   int peekFrontIndex()
   {
      return indices.peekFirst();
   }

   /**
    * Inserts {@code newIndex} (value {@code values[newIndex]}) as the new back entry, first popping any
    * existing back entries that are dominated -- value-wise no better than {@code newValue} and strictly
    * older, so they can never again be the extremum while {@code newIndex} is in the active window. On a
    * tie the older entry is popped (the kept entry is the newer one with an equal value), which is what
    * keeps this correct for duplicate-heavy series such as booleans and enums.
    *
    * @param newIndex must be chronologically after every index currently held (caller inserts in
    *                 increasing ring-buffer-traversal order, both within one tick and across ticks).
    */
   void pushCandidate(int newIndex, double newValue, double[] values)
   {
      while (!indices.isEmpty() && dominated(values[indices.peekLast()], newValue))
         indices.removeLast();
      if (indices.size() == ringBufferSize)
         throw new IllegalStateException("Deque capacity exceeded: more live candidates than ring buffer slots.");
      indices.addLast(newIndex);
   }

   /**
    * If the current front entry's ring-buffer index equals {@code evictedIndex}, pops it (front advances
    * to the next-oldest surviving candidate). No-op otherwise -- {@code evictedIndex} was already
    * dominated and dropped by an earlier {@link #pushCandidate}. Must be called once per index leaving
    * the active window, strictly in the order those indices age out (oldest first).
    */
   void evictIfFront(int evictedIndex)
   {
      if (!indices.isEmpty() && indices.peekFirst() == evictedIndex)
         indices.removeFirst();
   }

   /**
    * {@code <=}/{@code >=}, not {@code <}/{@code >}: on a tie the existing (older) entry is also
    * dominated and gets popped, so the newer entry of an equal pair is what's kept at the back. That's
    * what lets {@link #evictIfFront} work correctly for duplicate-heavy series (booleans, enums, a
    * constant double) -- the surviving candidate is always the one that ages out last.
    */
   private boolean dominated(double existing, double incoming)
   {
      return trackMax ? existing <= incoming : existing >= incoming;
   }
}
