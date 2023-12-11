package us.ihmc.scs2.session.mcap;

import gnu.trove.list.array.TLongArrayList;
import gnu.trove.map.hash.TLongObjectHashMap;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * This class is used to manage the chunks of an MCAP file.
 * <p>
 * The MCAP file is composed of a list of chunks. Each chunk contains a list of records. The records typically contains messages which are the actual data that
 * we want to read.
 * </p>
 * <p>
 * This class intends to simplify the access to the messages by automatically loading and unloading chunks as needed.
 * </p>
 */
public class MCAPChunkManager
{
   /**
    * The timestamps of all the messages in the MCAP file.
    */
   private final TLongArrayList allMessageTimestamps = new TLongArrayList();
   /**
    * All the chunk indices of the MCAP file.
    */
   private final List<MCAP.ChunkIndex> mcapChunkIndices = new ArrayList<>();
   private final long desiredLogDT;
   /**
    * The number of chunks currently loaded. It can either be 0, 1 or 2.
    */
   private int numberOfLoadedChunks = -1;
   /**
    * Wrapper for the currently loaded chunks.
    */
   private ChunkExtra loadedChunkA = new ChunkExtra();
   /**
    * Wrapper for the currently loaded chunks.
    */
   private ChunkExtra loadedChunkB = new ChunkExtra();

   public MCAPChunkManager(long desiredLogDT)
   {
      this.desiredLogDT = desiredLogDT;
   }

   /**
    * @return the timestamp of the message at the given index.
    */
   public long getTimestampAtIndex(int index)
   {
      return allMessageTimestamps.get(index);
   }

   /**
    * @return the timestamp of the message at the given index relative to the first message timestamp.
    */
   public long getRelativeTimestampAtIndex(int index)
   {
      return allMessageTimestamps.get(index) - firstMessageTimestamp();
   }

   /**
    * @return the index of the message with the given timestamp.
    */
   public int getIndexFromTimestamp(long timestamp)
   {
      int index = allMessageTimestamps.binarySearch(timestamp);
      if (index < 0)
         index = -(index + 1);
      return index;
   }

   /**
    * @return the number of messages in the MCAP file.
    */
   public int getNumberOfEntries()
   {
      return allMessageTimestamps.size();
   }

   /**
    * Initializes this manager from the given MCAP file.
    */
   public void loadFromMCAP(MCAP mcap)
   {
      for (MCAP.Record record : mcap.records())
      {
         if (record.op() == MCAP.Opcode.CHUNK_INDEX)
         {
            mcapChunkIndices.add((MCAP.ChunkIndex) record.body());
         }
         else if (record.op() == MCAP.Opcode.MESSAGE_INDEX)
         {
            MCAP.MessageIndex messageIndex = (MCAP.MessageIndex) record.body();
            for (MCAP.MessageIndex.MessageIndexEntry mcapEntry : messageIndex.records())
            {
               long timestamp = round(mcapEntry.logTime(), desiredLogDT);
               if (allMessageTimestamps.isEmpty() || timestamp > allMessageTimestamps.get(allMessageTimestamps.size() - 1))
               {
                  allMessageTimestamps.add(timestamp);
               }
               else if (timestamp == allMessageTimestamps.get(allMessageTimestamps.size() - 1))
               {
                  // Nothing, the timestamp is already there.
               }
               else
               {
                  int insertion = allMessageTimestamps.binarySearch(timestamp);
                  if (insertion < 0)
                  {
                     allMessageTimestamps.insert(-insertion - 1, timestamp);
                  }
                  else
                  {
                     // Nothing, the timestamp is already there.
                  }
               }
            }
            record.unloadBody(); // Unload the message index to save memory. TODO Investigate memory usage.
         }
      }
      // TODO The underlying algorithm seems quite expensive.
      mcapChunkIndices.sort(Comparator.comparingLong(chunkIndex -> round(chunkIndex.messageStartTime(), desiredLogDT)));
   }

   /**
    * @return the timestamp of the first message in the MCAP file.
    */
   public long firstMessageTimestamp()
   {
      return round(mcapChunkIndices.get(0).messageStartTime(), desiredLogDT);
   }

   /**
    * @return the timestamp of the last message in the MCAP file.
    */
   public long lastMessageTimestamp()
   {
      return round(mcapChunkIndices.get(mcapChunkIndices.size() - 1).messageEndTime(), desiredLogDT);
   }

   /**
    * @return the timestamp of the next message after the given timestamp.
    */
   public long nextMessageTimestamp(long timestamp)
   {
      if (timestamp < allMessageTimestamps.get(0))
         return allMessageTimestamps.get(0);
      if (timestamp >= allMessageTimestamps.get(allMessageTimestamps.size() - 1))
         return -1;

      int index = allMessageTimestamps.binarySearch(timestamp);
      if (index < 0)
         index = -(index + 1) + 1;
      else
         index++;

      return allMessageTimestamps.get(index);
   }

   public long previousMessageTimestamp(long timestamp)
   {
      if (timestamp <= allMessageTimestamps.get(0))
         return -1;
      if (timestamp > allMessageTimestamps.get(allMessageTimestamps.size() - 1))
         return allMessageTimestamps.get(allMessageTimestamps.size() - 1);

      int index = allMessageTimestamps.binarySearch(timestamp);
      if (index < 0)
         index = -(index + 1);
      else
         index--;

      return allMessageTimestamps.get(index);
   }

   /**
    * @return retrieves the messages at the given timestamp.
    */
   public List<MCAP.Message> loadMessages(long timestamp) throws IOException
   {
      if (timestamp <= getActiveChunkStartTimestamp() || timestamp >= getActiveChunkEndTimestamp())
         loadChunk(timestamp);

      // TODO Naive approach, improve later
      if (numberOfLoadedChunks <= 0)
         return Collections.emptyList();

      if (numberOfLoadedChunks == 1)
      {
         return loadedChunkA.bundledMessages.get(timestamp);
      }

      if (numberOfLoadedChunks == 2)
      {
         List<MCAP.Message> bundleMessagesA = loadedChunkA.bundledMessages.get(timestamp);
         List<MCAP.Message> bundleMessagesB = loadedChunkB.bundledMessages.get(timestamp);
         int length = 0;
         if (bundleMessagesA != null)
            length += bundleMessagesA.size();
         if (bundleMessagesB != null)
            length += bundleMessagesB.size();

         List<MCAP.Message> messages = new ArrayList<>(length);
         if (bundleMessagesA != null)
            messages.addAll(bundleMessagesA);
         if (bundleMessagesB != null)
            messages.addAll(bundleMessagesB);
         return messages;
      }

      throw new RuntimeException("Unexpected number of chunks: " + numberOfLoadedChunks);
   }

   /**
    * Loads the chunk that contains the given timestamp.
    * <p>
    * If the given timestamp is not contained in the MCAP file, this method will return {@code false} and the manager will be empty.
    * </p>
    *
    * @param timestamp the timestamp of the message to load.
    * @return {@code true} if the chunk was successfully loaded, {@code false} otherwise.
    */
   public boolean loadChunk(long timestamp) throws IOException
   {
      int index;
      if (timestamp == getActiveChunkEndTimestamp()) // When reading, this will be the most common case, no need to trigger a binary search.
         index = numberOfLoadedChunks == 1 ? loadedChunkA.index : loadedChunkB.index;
      else
         index = searchMCAPChunkIndex(timestamp);

      if (index < 0)
      {
         numberOfLoadedChunks = 0;
         return false;
      }

      int prevIndexA = loadedChunkA.index;
      int prevIndexB = loadedChunkB.index;
      int indexA, indexB = -1;

      MCAP.ChunkIndex mcapChunkIndex = mcapChunkIndices.get(index);

      if (timestamp == round(mcapChunkIndex.messageStartTime(), desiredLogDT) && index > 0)
      { // We are in between 2 chunks
         indexA = index - 1;
         indexB = index;
         // Updating the active chunks at the end of this method.
      }
      else if (timestamp == round(mcapChunkIndex.messageEndTime(), desiredLogDT) && index < mcapChunkIndices.size() - 1)
      { // We are in between 2 chunks
         indexA = index;
         indexB = index + 1;
         // Updating the active chunks at the end of this method.
      }
      else
      { // We are in a single chunk
         numberOfLoadedChunks = 1;

         if (index == prevIndexA)
            return true; // Nothing's changed
         if (index == prevIndexB)
         {
            swapChunks();
            loadedChunkB.clear();
            if (prevIndexA > -1) // Unload the previous chunk in prevIndexA
               mcapChunkIndices.get(prevIndexA).unloadChunk();
            return true;
         }

         if (prevIndexB > -1)
         {
            loadedChunkB.clear();
            // Unload the previous chunk in prevIndexB
            mcapChunkIndices.get(prevIndexB).unloadChunk();
         }
         loadedChunkA.clear();
         loadedChunkA.index = index;
         loadedChunkA.initialize((MCAP.Chunk) mcapChunkIndex.chunk().body());
         if (prevIndexA > -1) // Unload the previous chunk in prevIndexA
            mcapChunkIndices.get(prevIndexA).unloadChunk();
         return true;
      }

      numberOfLoadedChunks = 2;
      MCAP.ChunkIndex mcapChunkIndexA = mcapChunkIndices.get(indexA);
      MCAP.ChunkIndex mcapChunkIndexB = mcapChunkIndices.get(indexB);

      if (indexA == prevIndexB || indexB == prevIndexA)
      { // Swapping the chunks and resume assuming the chunks cannot be swapped. That should simplify a little.
         swapChunks();
         int tmp = prevIndexA;
         prevIndexA = prevIndexB;
         prevIndexB = tmp;
      }

      // We have 2 active chunks
      if (indexA == prevIndexA)
      {
         if (indexB == prevIndexB)
            return true; // Nothing's changed

         // indexB != prevIndexB
         loadedChunkB.clear();
         loadedChunkB.index = indexB;
         loadedChunkB.initialize((MCAP.Chunk) mcapChunkIndexB.chunk().body());
         if (prevIndexB > -1) // Unload the previous chunk in prevIndexB
            mcapChunkIndices.get(prevIndexB).unloadChunk();
      }
      else if (indexB == prevIndexB)
      { // indexA != prevIndexA
         loadedChunkA.clear();
         loadedChunkA.index = indexA;
         loadedChunkA.initialize((MCAP.Chunk) mcapChunkIndexA.chunk().body());
         if (prevIndexA > -1) // Unload the previous chunk in prevIndexA
            mcapChunkIndices.get(prevIndexA).unloadChunk();
      }
      else
      { // indexA != prevIndexA && indexB != prevIndexB
         loadedChunkA.clear();
         loadedChunkA.index = indexA;
         loadedChunkA.initialize((MCAP.Chunk) mcapChunkIndexA.chunk().body());
         loadedChunkB.clear();
         loadedChunkB.index = indexB;
         loadedChunkB.initialize((MCAP.Chunk) mcapChunkIndexB.chunk().body());
         // Unload the previous chunks in prevIndexA and prevIndexB
         if (prevIndexA > -1)
            mcapChunkIndices.get(prevIndexA).unloadChunk();
         if (prevIndexB > -1)
            mcapChunkIndices.get(prevIndexB).unloadChunk();
      }

      return true;
   }

   private void swapChunks()
   {
      ChunkExtra temp = loadedChunkA;
      loadedChunkA = loadedChunkB;
      loadedChunkB = temp;
   }

   /**
    * @return the timestamp of the first message in the currently loaded chunk.
    */
   public long getActiveChunkStartTimestamp()
   {
      if (numberOfLoadedChunks <= 0)
         return -1;
      else
         return round(loadedChunkA.chunk.messageStartTime(), desiredLogDT);
   }

   /**
    * @return the timestamp of the last message in the currently loaded chunk.
    */
   public long getActiveChunkEndTimestamp()
   {
      if (numberOfLoadedChunks <= 0)
         return -1;
      else if (numberOfLoadedChunks == 1)
         return round(loadedChunkA.chunk.messageEndTime(), desiredLogDT);
      else if (numberOfLoadedChunks == 2)
         return round(loadedChunkB.chunk.messageEndTime(), desiredLogDT);
      else
         throw new RuntimeException("Unexpected number of chunks: " + numberOfLoadedChunks);
   }

   private int searchMCAPChunkIndex(long timestamp)
   {
      if (mcapChunkIndices.isEmpty())
         return -1;

      int low = 0;
      int high = mcapChunkIndices.size() - 1;

      if (timestamp < round(mcapChunkIndices.get(low).messageStartTime(), desiredLogDT))
         return -1;
      if (timestamp > round(mcapChunkIndices.get(high).messageEndTime(), desiredLogDT))
         return -1;

      while (low <= high)
      {
         int mid = (low + high) >>> 1;
         MCAP.ChunkIndex midVal = mcapChunkIndices.get(mid);
         long midValStartTime = round(midVal.messageStartTime(), desiredLogDT);

         if (timestamp == midValStartTime)
            return mid;

         if (timestamp > midValStartTime)
         {
            if (timestamp <= round(midVal.messageEndTime(), desiredLogDT))
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

   /**
    * Rounds the given value to the nearest multiple of the given step.
    *
    * @param value the value to round.
    * @param step  the step to round to.
    * @return the rounded value.
    */
   static long round(long value, long step)
   {
      if (step <= 1)
         return value;
      long floor = value / step * step;
      long ceil = floor + step;
      return value - floor < ceil - value ? floor : ceil;
   }

   private class ChunkExtra
   {
      /**
       * The index of the chunk in mcapChunkIndices.
       */
      private int index = -1;
      /**
       * The chunk currently loaded.
       */
      private MCAP.Chunk chunk;
      /**
       * Messages bundle per their timestamp.
       */
      private final TLongObjectHashMap<List<MCAP.Message>> bundledMessages = new TLongObjectHashMap<>();

      private void clear()
      {
         index = -1;
         chunk = null;
         bundledMessages.clear();
      }

      public void initialize(MCAP.Chunk chunk) throws IOException
      {
         this.chunk = chunk;

         for (MCAP.Record record : chunk.records())
         {
            if (record.op() != MCAP.Opcode.MESSAGE)
               continue;

            MCAP.Message message = (MCAP.Message) record.body();
            List<MCAP.Message> messages = bundledMessages.get(round(message.logTime(), desiredLogDT));
            if (messages == null)
            {
               messages = new ArrayList<>();
               bundledMessages.put(round(message.logTime(), desiredLogDT), messages);
            }
            messages.add(message);
         }
      }
   }
}
