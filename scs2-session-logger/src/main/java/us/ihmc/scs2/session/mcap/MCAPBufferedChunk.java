package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.ChunkIndex;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.RecordDataInputBacked;
import us.ihmc.scs2.session.mcap.specs.records.Records;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.scs2.session.mcap.MCAPMessageManager.round;

/**
 * This class is used to identify the chunks that need to be loaded and how many chunks can be loaded at the same time.
 */
public class MCAPBufferedChunk
{
   private static final double ALLOWABLE_CHUNK_MEMORY_RATIO = 0.05;
   private final MCAP mcap;
   private final long desiredLogDT;
   private final int maxNumberOfChunksLoaded;

   private final ConcurrentLinkedQueue<ChunkBundle> loadedChunkBundles = new ConcurrentLinkedQueue<>();
   private final ChunkBundle[] chunkBundles;
   private final ExecutorService executorService = Executors.newFixedThreadPool(4, ThreadTools.createNamedDaemonThreadFactory(getClass().getSimpleName()));

   public MCAPBufferedChunk(MCAP mcap, long desiredLogDT)
   {
      this.mcap = mcap;
      this.desiredLogDT = desiredLogDT;

      int numberOfChunks = 0;
      long minChunkSize = Long.MAX_VALUE;
      long maxChunkSize = Long.MIN_VALUE;
      long totalChunkSize = 0;

      List<ChunkIndex> orderedChunkIndices = new ArrayList<>();

      for (Record record : mcap.records())
      {
         if (record.op() == Opcode.CHUNK)
         {
            Chunk chunk = (Chunk) record.body();
            numberOfChunks++;
            long chunkSize = chunk.recordsCompressedLength();
            minChunkSize = Math.min(minChunkSize, chunkSize);
            maxChunkSize = Math.max(maxChunkSize, chunkSize);
            totalChunkSize += chunkSize;
         }
         else if (record.op() == Opcode.CHUNK_INDEX)
         {
            ChunkIndex chunkIndex = (ChunkIndex) record.body();
            orderedChunkIndices.add(chunkIndex);
         }
      }

      long averageChunkSize = totalChunkSize / numberOfChunks;
      maxNumberOfChunksLoaded = (int) Math.ceil(ALLOWABLE_CHUNK_MEMORY_RATIO * Runtime.getRuntime().maxMemory() / averageChunkSize);
      LogTools.info("Chunk stats: [Average size: %d, min size: %d, max size: %d, total size: %d, quantity: %d], max memory: %d, max chunks loaded: %d".formatted(
            averageChunkSize,
            minChunkSize,
            maxChunkSize,
            totalChunkSize,
            numberOfChunks,
            Runtime.getRuntime().maxMemory(),
            maxNumberOfChunksLoaded));

      orderedChunkIndices.sort(Comparator.comparingLong(chunkIndex -> round(chunkIndex.messageStartTime(), desiredLogDT)));

      chunkBundles = new ChunkBundle[numberOfChunks];
      for (int i = 0; i < numberOfChunks; i++)
      {
         chunkBundles[i] = new ChunkBundle(i, orderedChunkIndices.get(i));
      }
   }

   public ChunkBundle[] getChunkBundles()
   {
      return chunkBundles;
   }

   public ChunkBundle getChunkBundle(long logTime)
   {
      int chunkIndex = searchChunkBundle(logTime);
      if (chunkIndex < 0)
         return null;
      else
         return chunkBundles[chunkIndex];
   }

   public void requestLoadChunk(long logTime, boolean wait)
   {
      ChunkBundle chunkBundle = getChunkBundle(logTime);
      if (chunkBundle != null)
         chunkBundle.requestLoadChunkBundle(wait);
   }

   public void preloadChunks(long startTime, long duration)
   {
      ChunkBundle chunkBundle = getChunkBundle(startTime);
      if (chunkBundle == null)
         return;

      int maxNumberOfChunksToLoad = maxNumberOfChunksLoaded / 2;
      int numberOfChunksLoaded = 1;
      chunkBundle.requestLoadChunkBundle(false);

      while (chunkBundle.endTime() < startTime + duration && numberOfChunksLoaded < maxNumberOfChunksToLoad)
      {
         chunkBundle = chunkBundle.next();

         if (chunkBundle == null)
            break;

         numberOfChunksLoaded++;
         chunkBundle.requestLoadChunkBundle(false);
      }
   }

   public int getMaxNumberOfChunksLoaded()
   {
      return maxNumberOfChunksLoaded;
   }

   public int getNumberOfChunksLoaded()
   {
      return loadedChunkBundles.size();
   }

   private int searchChunkBundle(long timestamp)
   {
      if (chunkBundles.length == 0)
         return -1;

      int low = 0;
      int high = chunkBundles.length - 1;

      if (timestamp < chunkBundles[low].startTime())
         return -1;
      if (timestamp > chunkBundles[high].endTime())
         return -1;

      while (low <= high)
      {
         int mid = (low + high) >>> 1;
         ChunkBundle midVal = chunkBundles[mid];
         long midValStartTime = midVal.startTime();

         if (timestamp == midValStartTime)
            return mid;

         if (timestamp > midValStartTime)
         {
            if (timestamp <= midVal.endTime())
               return mid;
            else
               low = mid + 1;
         }
         else
         {
            high = mid - 1;
         }
      }
      return -1;
   }

   public ChunkBundle getChunkBundle(int chunkIndex)
   {
      return chunkBundles[chunkIndex];
   }

   public int getNumberOfChunks()
   {
      return chunkBundles.length;
   }

   /**
    * Atomically claims responsibility for loading a value into {@code futureRef}, memoizer-style (Java Concurrency in Practice, Goetz,
    * listing 5.19): if nobody has started yet, this installs a fresh, not-yet-completed future and returns it -- the caller is now on the
    * hook to run the load and complete it. If someone else already started (or finished), this returns {@code null} and does nothing; the
    * existing future (available via {@code futureRef.get()}) is the one to use instead.
    */
   private static <T> CompletableFuture<T> claim(AtomicReference<CompletableFuture<T>> futureRef)
   {
      CompletableFuture<T> newFuture = new CompletableFuture<>();
      return futureRef.compareAndSet(null, newFuture) ? newFuture : null;
   }

   /** Joins a future, unwrapping a checked {@link IOException} cause so callers that declare {@code throws IOException} can keep doing so. */
   private static <T> T awaitChecked(CompletableFuture<T> future) throws IOException
   {
      try
      {
         return future.join();
      }
      catch (CompletionException e)
      {
         Throwable cause = e.getCause();
         if (cause instanceof IOException ioException)
            throw ioException;
         if (cause instanceof RuntimeException runtimeException)
            throw runtimeException;
         throw new RuntimeException(cause);
      }
   }

   /** Joins a future for callers that don't declare any checked exception, matching how InterruptedException used to be handled inline. */
   private static <T> T awaitUnchecked(CompletableFuture<T> future)
   {
      try
      {
         return future.join();
      }
      catch (CompletionException e)
      {
         Throwable cause = e.getCause();
         throw cause instanceof RuntimeException runtimeException ? runtimeException : new RuntimeException(cause);
      }
   }

   public class ChunkBundle
   {
      private final int index;
      private final ChunkIndex chunkIndex;

      // Memoizer pattern (Java Concurrency in Practice, Goetz, listing 5.19): a chunk's records/messages are each loaded at most once, by
      // whichever thread wins the compareAndSet in claim(...); any other thread that asks -- concurrently or later -- gets (and can join())
      // that same future instead of redoing the work or racing on shared mutable state.
      // This replaces a hand-rolled latch+field pair per value with a single object that carries "is it done", "what's the result", and "did it fail" together,
      // which is what used to be maintained by hand (and got out of sync under concurrent access: two threads racing into the same TLongObjectHashMap.put() loop
      // corrupted trove's internal table and threw out of rehash()).
      private final AtomicReference<CompletableFuture<Records>> chunkRecordsFuture = new AtomicReference<>();
      private final AtomicReference<CompletableFuture<TLongObjectHashMap<List<Message>>>> bundledMessagesFuture = new AtomicReference<>();

      private long lastLoadingRequestTime = Long.MIN_VALUE;

      public ChunkBundle(int index, ChunkIndex chunkIndex)
      {
         this.index = index;
         this.chunkIndex = chunkIndex;
      }

      public ChunkIndex getChunkIndex()
      {
         return chunkIndex;
      }

      public ChunkBundle next()
      {
         return index + 1 < chunkBundles.length ? chunkBundles[index + 1] : null;
      }

      public ChunkBundle previous()
      {
         return index > 0 ? chunkBundles[index - 1] : null;
      }

      private void freeUpChunkBundleSpots(int numberOfSpots)
      {
         while (loadedChunkBundles.size() > maxNumberOfChunksLoaded - numberOfSpots)
         {
            ChunkBundle oldestChunkBundle = null;
            for (ChunkBundle chunkBundle : loadedChunkBundles)
            {
               if (oldestChunkBundle == null || chunkBundle.lastLoadingRequestTime < oldestChunkBundle.lastLoadingRequestTime)
                  oldestChunkBundle = chunkBundle;
            }
            if (oldestChunkBundle == null)
               throw new RuntimeException("Unexpected: no chunk bundle to unload");
            oldestChunkBundle.unloadChunk();
         }
      }

      private void unloadChunk()
      {
         // Simple field resets, each independently atomic (AtomicReference.set()) -- no synchronized needed. A future object already handed
         // out to another thread (e.g. one it's mid-join() on) stays valid and correct for that thread regardless of what these fields get
         // reset to afterward; resetting them here only affects what the *next* request sees.
         chunkRecordsFuture.set(null);
         bundledMessagesFuture.set(null);
         loadedChunkBundles.remove(this);
         lastLoadingRequestTime = Long.MIN_VALUE;
      }

      public void requestLoadChunkBundle(final boolean wait)
      {
         requestLoadChunkBundle(wait, true, true);
      }

      /**
       * Requests the chunk to be loaded.
       *
       * @param wait              whether to wait for the chunk to be loaded before returning.
       * @param recordRequestTime whether to record the time at which the request was made.
       * @param createMessages    whether to create the messages from the chunk.
       */
      public void requestLoadChunkBundle(final boolean wait, final boolean recordRequestTime, final boolean createMessages)
      {
         if (recordRequestTime)
            lastLoadingRequestTime = System.nanoTime();

         CompletableFuture<Records> claimedFuture = claim(chunkRecordsFuture);
         CompletableFuture<Records> recordsFuture = claimedFuture != null ? claimedFuture : chunkRecordsFuture.get();

         if (claimedFuture != null)
         {
            // Nobody was loading this chunk yet -- we are now responsible for it (and its messages, if requested).
            freeUpChunkBundleSpots(1);
            Runnable loadingTask = () -> runChunkLoad(claimedFuture, createMessages);

            if (wait)
               loadingTask.run();
            else
               executorService.submit(loadingTask);
         }
         else if (!wait && createMessages && recordsFuture != null && recordsFuture.isDone() && !recordsFuture.isCompletedExceptionally())
         {
            // The chunk itself is already loaded (by an earlier request), but nobody has necessarily started on the messages yet -- mirrors
            // the old behavior of checking this on every request, not just the first one, without blocking since wait == false here.
            startLoadingMessagesAsync();
         }

         if (wait)
         {
            if (recordsFuture != null)
               awaitUnchecked(recordsFuture);
            if (createMessages)
            {
               try
               {
                  loadMessagesNow();
               }
               catch (IOException e)
               {
                  throw new RuntimeException(e);
               }
            }
         }
      }

      /**
       * Runs on whichever thread ended up responsible for loading this chunk (the calling thread if wait == true, an executor thread
       * otherwise): loads the chunk's records, then its messages if requested, unloading the chunk entirely on any failure so a later
       * request retries from scratch instead of being stuck with a permanently-failed or partial state.
       */
      private void runChunkLoad(CompletableFuture<Records> future, boolean createMessages)
      {
         try
         {
            ByteBuffer chunkBuffer = mcap.getDataInput().getByteBuffer(chunkIndex.chunkOffset(), (int) chunkIndex.chunkLength(), true);
            Records records = ((Chunk) new RecordDataInputBacked(MCAPDataInput.wrap(chunkBuffer), 0).body()).records();

            if (!loadedChunkBundles.contains(this))
               loadedChunkBundles.add(this);

            future.complete(records);

            if (createMessages)
               loadMessagesNow();
         }
         catch (Exception e)
         {
            e.printStackTrace();
            future.completeExceptionally(e); // no-op if future.complete(records) above already succeeded -- only the messages step failed
            unloadChunk();
         }
      }

      /** Kicks off a message load if nobody has started one yet, without waiting for it to finish. */
      private void startLoadingMessagesAsync()
      {
         CompletableFuture<TLongObjectHashMap<List<Message>>> claimedFuture = claim(bundledMessagesFuture);
         if (claimedFuture != null)
            executorService.submit(() -> runMessagesLoad(claimedFuture));
      }

      public TLongObjectHashMap<List<Message>> loadMessagesNow() throws IOException
      {
         CompletableFuture<TLongObjectHashMap<List<Message>>> claimedFuture = claim(bundledMessagesFuture);
         CompletableFuture<TLongObjectHashMap<List<Message>>> future = claimedFuture != null ? claimedFuture : bundledMessagesFuture.get();

         if (claimedFuture != null)
            runMessagesLoad(claimedFuture);

         return awaitChecked(future);
      }

      private void runMessagesLoad(CompletableFuture<TLongObjectHashMap<List<Message>>> future)
      {
         try
         {
            // Built into a local variable that no other thread can see or touch until fully populated, then published to the future in one
            // shot via complete(...) -- as opposed to mutating a shared field in place while other threads might be reading/writing it too.
            TLongObjectHashMap<List<Message>> newBundledMessages = new TLongObjectHashMap<>();

            for (Record record : chunkRecordsFuture.get().join())
            {
               if (record.op() != Opcode.MESSAGE)
                  continue;

               Message message = record.body();
               List<Message> messages = newBundledMessages.get(round(message.logTime(), desiredLogDT));
               if (messages == null)
               {
                  messages = new ArrayList<>();
                  newBundledMessages.put(round(message.logTime(), desiredLogDT), messages);
               }
               messages.add(message);
            }

            future.complete(newBundledMessages);
         }
         catch (Exception e)
         {
            future.completeExceptionally(e);
         }
      }

      public Records getChunkRecords()
      {
         CompletableFuture<Records> future = chunkRecordsFuture.get();
         if (future == null || !future.isDone() || future.isCompletedExceptionally())
            return null;
         return future.getNow(null);
      }

      public long startTime()
      {
         return round(chunkIndex.messageStartTime(), desiredLogDT);
      }

      public long endTime()
      {
         return round(chunkIndex.messageEndTime(), desiredLogDT);
      }

      public List<Message> getMessages(long logTime)
      {
         if (getChunkRecords() == null)
            return null;

         TLongObjectHashMap<List<Message>> messages;
         try
         {
            messages = loadMessagesNow();
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
         return messages.get(round(logTime, desiredLogDT));
      }
   }
}
