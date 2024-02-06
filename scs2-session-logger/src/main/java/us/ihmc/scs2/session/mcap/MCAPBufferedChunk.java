package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAP.Chunk;
import us.ihmc.scs2.session.mcap.MCAP.ChunkIndex;
import us.ihmc.scs2.session.mcap.MCAP.Message;
import us.ihmc.scs2.session.mcap.MCAP.Opcode;
import us.ihmc.scs2.session.mcap.MCAP.Record;
import us.ihmc.scs2.session.mcap.MCAP.Records;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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
            long chunkSize = chunk.recordsLength();
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

   public class ChunkBundle
   {
      private final int index;
      private final ChunkIndex chunkIndex;
      private Records chunkRecords;
      private TLongObjectHashMap<List<Message>> bundledMessages;

      private volatile CountDownLatch chunkLoadedLatch, messagesLoadedLatch;

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
         chunkRecords = null;
         bundledMessages = null;
         chunkLoadedLatch = null;
         messagesLoadedLatch = null;
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

         if (chunkRecords != null)
         {
            if (createMessages && bundledMessages == null)
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
            return;
         }

         if (chunkLoadedLatch == null)
         {
            chunkLoadedLatch = new CountDownLatch(1);
            freeUpChunkBundleSpots(1);

            Runnable loadingTask = () ->
            {
               try
               {
                  loadChunkNow();
                  if (createMessages)
                     loadMessagesNow();
               }
               catch (Exception e)
               {
                  e.printStackTrace();
                  unloadChunk();
               }
               finally
               {
                  chunkLoadedLatch.countDown();
                  chunkLoadedLatch = null;
               }
            };

            if (wait)
               loadingTask.run();
            else
               executorService.submit(loadingTask);
         }

         try
         {
            if (chunkLoadedLatch != null && wait)
               chunkLoadedLatch.await();
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException(e);
         }
      }

      private void loadChunkNow() throws IOException
      {
         if (chunkRecords == null)
         {
            ByteBuffer chunkBuffer = mcap.getDataInput().getByteBuffer(chunkIndex.chunkOffset(), (int) chunkIndex.chunkLength(), true);
            chunkRecords = ((Chunk) new Record(MCAPDataInput.wrap(chunkBuffer), 0).body()).records();
         }

         if (!loadedChunkBundles.contains(this))
            loadedChunkBundles.add(this);
      }

      public void loadMessagesNow() throws IOException
      {
         if (bundledMessages != null)
            return;

         if (messagesLoadedLatch != null)
         {
            try
            {
               messagesLoadedLatch.await();
            }
            catch (InterruptedException e)
            {
               throw new RuntimeException(e);
            }
            return;
         }

         messagesLoadedLatch = new CountDownLatch(1);

         try
         {
            for (Record record : chunkRecords)
            {
               if (record.op() != Opcode.MESSAGE)
                  continue;

               if (bundledMessages == null)
                  bundledMessages = new TLongObjectHashMap<>();

               Message message = (Message) record.body();
               List<Message> messages = bundledMessages.get(round(message.logTime(), desiredLogDT));
               if (messages == null)
               {
                  messages = new ArrayList<>();
                  bundledMessages.put(round(message.logTime(), desiredLogDT), messages);
               }
               messages.add(message);
            }
         }
         finally
         {
            messagesLoadedLatch.countDown();
            messagesLoadedLatch = null;
         }
      }

      public Records getChunkRecords()
      {
         return chunkRecords;
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
         if (chunkRecords == null)
            return null;

         if (bundledMessages == null)
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
         return bundledMessages.get(round(logTime, desiredLogDT));
      }
   }
}
