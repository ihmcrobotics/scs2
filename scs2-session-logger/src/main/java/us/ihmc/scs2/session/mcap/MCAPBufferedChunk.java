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
import java.util.concurrent.ConcurrentLinkedQueue;
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

   public class ChunkBundle
   {
      private final int index;
      private final ChunkIndex chunkIndex;
      private volatile Records chunkRecords;
      private volatile TLongObjectHashMap<List<Message>> bundledMessages;

      private final Object chunkLoadedLock = new Object();
      private final Object messagesLoadedLock = new Object();
      private boolean chunkLoading = false;
      private boolean messagesLoading = false;

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
               if (chunkBundle.isLoading())
                  continue;

               if (oldestChunkBundle == null || chunkBundle.lastLoadingRequestTime < oldestChunkBundle.lastLoadingRequestTime)
                  oldestChunkBundle = chunkBundle;
            }

            if (oldestChunkBundle == null)
               return;

            oldestChunkBundle.unloadChunk();
         }
      }

      private void unloadChunk()
      {
         synchronized (chunkLoadedLock)
         {
            synchronized (messagesLoadedLock)
            {
               bundledMessages = null;
               messagesLoading = false;
               messagesLoadedLock.notifyAll();
            }

            chunkRecords = null;
            chunkLoading = false;
            chunkLoadedLock.notifyAll();
         }
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

         boolean loadChunk = false;
         boolean loadMessages = false;

         synchronized (chunkLoadedLock)
         {
            if (chunkRecords != null)
            {
               loadMessages = createMessages && bundledMessages == null;
            }
            else if (!chunkLoading)
            {
               chunkLoading = true;
               loadChunk = true;
            }
         }

         if (loadMessages)
         {
            try
            {
               loadMessagesNow();
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
            return;
         }

         if (loadChunk)
         {
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
                  synchronized (chunkLoadedLock)
                  {
                     chunkLoading = false;
                     chunkLoadedLock.notifyAll();
                  }
               }
            };

            if (wait)
               loadingTask.run();
            else
               executorService.submit(loadingTask);
         }

         if (wait)
         {
            waitForChunkLoaded();

            if (createMessages && chunkRecords != null && bundledMessages == null)
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

      private void loadChunkNow() throws IOException
      {
         if (chunkRecords == null)
         {
            ByteBuffer chunkBuffer = mcap.getDataInput().getByteBuffer(chunkIndex.chunkOffset(), (int) chunkIndex.chunkLength(), true);
            chunkRecords = ((Chunk) new RecordDataInputBacked(MCAPDataInput.wrap(chunkBuffer), 0).body()).records();
         }

         if (!loadedChunkBundles.contains(this))
            loadedChunkBundles.add(this);
      }

      public void loadMessagesNow() throws IOException
      {
         Records records = chunkRecords;
         if (records == null)
         {
            requestLoadChunkBundle(true, false, false);
            records = chunkRecords;
            if (records == null)
               return;
         }

         synchronized (messagesLoadedLock)
         {
            while (messagesLoading)
            {
               try
               {
                  messagesLoadedLock.wait();
               }
               catch (InterruptedException e)
               {
                  throw new RuntimeException(e);
               }
            }

            if (bundledMessages != null)
               return;

            messagesLoading = true;
         }

         TLongObjectHashMap<List<Message>> newBundledMessages = new TLongObjectHashMap<>();

         try
         {
            for (Record record : records)
            {
               if (record.op() != Opcode.MESSAGE)
                  continue;

               Message message = record.body();
               long roundedLogTime = round(message.logTime(), desiredLogDT);
               List<Message> messages = newBundledMessages.get(roundedLogTime);
               if (messages == null)
               {
                  messages = new ArrayList<>();
                  newBundledMessages.put(roundedLogTime, messages);
               }
               messages.add(message);
            }

            synchronized (messagesLoadedLock)
            {
               if (chunkRecords == records && bundledMessages == null)
                  bundledMessages = newBundledMessages;
            }
         }
         finally
         {
            synchronized (messagesLoadedLock)
            {
               messagesLoading = false;
               messagesLoadedLock.notifyAll();
            }
         }
      }

      private void waitForChunkLoaded()
      {
         synchronized (chunkLoadedLock)
         {
            while (chunkLoading)
            {
               try
               {
                  chunkLoadedLock.wait();
               }
               catch (InterruptedException e)
               {
                  throw new RuntimeException(e);
               }
            }
         }
      }

      private boolean isLoading()
      {
         synchronized (chunkLoadedLock)
         {
            synchronized (messagesLoadedLock)
            {
               return chunkLoading || messagesLoading;
            }
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

         TLongObjectHashMap<List<Message>> messagesByLogTime = bundledMessages;
         return messagesByLogTime == null ? null : messagesByLogTime.get(round(logTime, desiredLogDT));
      }
   }
}
