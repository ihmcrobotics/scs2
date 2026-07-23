package us.ihmc.scs2.sessionVisualizer.jfx.charts;

/**
 * Fixed-capacity circular deque of ring-buffer indices, maintained so that the entry at the front is
 * always the current extremum (max, or min, depending on {@code trackMax}) candidate over the set of
 * indices currently held. Backed by a single {@code int[]} sized to the ring buffer's capacity, so
 * steady-state push/pop is allocation-free. Pure index/array logic, no JavaFX or SCS2 buffer types,
 * so it is unit-testable in isolation without the JavaFX toolkit.
 */
class MonotonicIndexDeque
{
   private final int[] indices;
   private final boolean trackMax;
   private int head;
   private int size;

   MonotonicIndexDeque(int ringBufferSize, boolean trackMax)
   {
      indices = new int[ringBufferSize];
      this.trackMax = trackMax;
   }

   void clear()
   {
      head = 0;
      size = 0;
   }

   boolean isEmpty()
   {
      return size == 0;
   }

   int size()
   {
      return size;
   }

   /** Ring-buffer index of the current extremum candidate. Caller must check {@link #isEmpty()} first. */
   int peekFrontIndex()
   {
      return indices[head];
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
      while (size > 0 && dominated(values[indices[tailSlot()]], newValue))
         size--;
      if (size == indices.length)
         throw new IllegalStateException("Deque capacity exceeded: more live candidates than ring buffer slots.");
      indices[(head + size) % indices.length] = newIndex;
      size++;
   }

   /**
    * If the current front entry's ring-buffer index equals {@code evictedIndex}, pops it (front advances
    * to the next-oldest surviving candidate). No-op otherwise -- {@code evictedIndex} was already
    * dominated and dropped by an earlier {@link #pushCandidate}. Must be called once per index leaving
    * the active window, strictly in the order those indices age out (oldest first).
    */
   void evictIfFront(int evictedIndex)
   {
      if (size > 0 && indices[head] == evictedIndex)
      {
         head = (head + 1) % indices.length;
         size--;
      }
   }

   private int tailSlot()
   {
      return (head + size - 1) % indices.length;
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
