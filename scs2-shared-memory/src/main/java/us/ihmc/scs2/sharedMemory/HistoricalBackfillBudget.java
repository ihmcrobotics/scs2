package us.ihmc.scs2.sharedMemory;

/**
 * How much time the buffer manager is willing to spend, in one publish cycle, pulling history out of a
 * {@link HistoricalValueBitsSource} - shared by every {@link YoVariableBuffer} that has such a source installed, so
 * the limit is on the total rather than per-buffer.
 * <p>
 * Backfilling runs on the buffer manager's own thread, in the middle of the tick that publishes data to consumers.
 * That thread is also the one that applies the user's requests to change the current buffer index, so any time spent
 * backfilling is time the UI spends unable to scrub or drag. For a log-backed source the work is unbounded in
 * principle - asking a freshly charted variable for its whole history means decompressing every batch of the log the
 * buffer spans - so it has to be spread across cycles rather than run to completion inside one.
 * </p>
 * <p>
 * The budget is wall-clock rather than a count of indices because the cost per index varies by orders of magnitude
 * with the log's compression, batch size and variable count, none of which the buffer layer knows about.
 * </p>
 */
public class HistoricalBackfillBudget
{
   /**
    * Default wall-clock allowance per publish cycle, in milliseconds. Publishing runs at roughly 30 Hz, so this
    * trades a chart taking a beat to fill in against the manager thread staying responsive to the user.
    */
   public static final long DEFAULT_BUDGET_MILLISECONDS = Long.getLong("scs2.buffer.backfill.budgetms", 5L);

   private long budgetNanoseconds = DEFAULT_BUDGET_MILLISECONDS * 1_000_000L;
   private long deadlineNanoseconds;

   public HistoricalBackfillBudget()
   {
      startCycle();
   }

   /**
    * @param budgetNanoseconds the allowance for each subsequent cycle. Zero or less removes the limit, which makes
    *                          backfilling run to completion within a single cycle the way it did before this existed.
    */
   public void setBudgetNanoseconds(long budgetNanoseconds)
   {
      this.budgetNanoseconds = budgetNanoseconds;
      startCycle();
   }

   /** Opens a fresh allowance. Called once per publish cycle by {@link YoSharedBuffer}. */
   public void startCycle()
   {
      deadlineNanoseconds = budgetNanoseconds <= 0L ? Long.MAX_VALUE : System.nanoTime() + budgetNanoseconds;
   }

   /** Whether this cycle's allowance is used up, i.e. any further backfilling should wait for the next cycle. */
   public boolean isExhausted()
   {
      return deadlineNanoseconds != Long.MAX_VALUE && System.nanoTime() >= deadlineNanoseconds;
   }
}
