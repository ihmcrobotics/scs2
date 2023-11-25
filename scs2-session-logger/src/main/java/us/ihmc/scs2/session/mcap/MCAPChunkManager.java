package us.ihmc.scs2.session.mcap;

import gnu.trove.list.array.TLongArrayList;
import gnu.trove.map.hash.TLongObjectHashMap;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MCAPChunkManager
{
   private final TLongArrayList allMessageTimestamps = new TLongArrayList();
   private final List<MCAP.ChunkIndex> mcapChunkIndices = new ArrayList<>();
   private int numberOfLoadedChunks = -1;
   private ChunkExtra loadedChunkA = new ChunkExtra();
   private ChunkExtra loadedChunkB = new ChunkExtra();

   public MCAPChunkManager()
   {
   }

   public long getTimestampAtIndex(int index)
   {
      return allMessageTimestamps.get(index);
   }

   public long getRelativeTimestampAtIndex(int index)
   {
      return allMessageTimestamps.get(index) - firstMessageTimestamp();
   }

   public int getIndexFromTimestamp(long timestamp)
   {
      int index = allMessageTimestamps.binarySearch(timestamp);
      if (index < 0)
         index = -(index + 1);
      return index;
   }

   public int getNumberOfEntries()
   {
      return allMessageTimestamps.size();
   }

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
               long timestamp = mcapEntry.logTime();
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
      Collections.sort(mcapChunkIndices, (c1, c2) -> Long.compare(c1.messageStartTime(), c2.messageStartTime()));
   }

   public long firstMessageTimestamp()
   {
      return mcapChunkIndices.get(0).messageStartTime();
   }

   public long lastMessageTimestamp()
   {
      return mcapChunkIndices.get(mcapChunkIndices.size() - 1).messageEndTime();
   }

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

      if (timestamp == mcapChunkIndex.messageStartTime() && index > 0)
      { // We are in between 2 chunks
         indexA = index - 1;
         indexB = index;
         // Updating the active chunks at the end of this method.
      }
      else if (timestamp == mcapChunkIndex.messageEndTime() && index < mcapChunkIndices.size() - 1)
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

   public long getActiveChunkStartTimestamp()
   {
      if (numberOfLoadedChunks <= 0)
         return -1;
      else
         return loadedChunkA.chunk.messageStartTime();
   }

   public long getActiveChunkEndTimestamp()
   {
      if (numberOfLoadedChunks <= 0)
         return -1;
      else if (numberOfLoadedChunks == 1)
         return loadedChunkA.chunk.messageEndTime();
      else if (numberOfLoadedChunks == 2)
         return loadedChunkB.chunk.messageEndTime();
      else
         throw new RuntimeException("Unexpected number of chunks: " + numberOfLoadedChunks);
   }

   private int searchMCAPChunkIndex(long timestamp)
   {
      if (mcapChunkIndices.isEmpty())
         return -1;

      int low = 0;
      int high = mcapChunkIndices.size() - 1;

      if (timestamp < mcapChunkIndices.get(low).messageStartTime())
         return -1;
      if (timestamp > mcapChunkIndices.get(high).messageEndTime())
         return -1;

      while (low <= high)
      {
         int mid = (low + high) >>> 1;
         MCAP.ChunkIndex midVal = mcapChunkIndices.get(mid);
         if (timestamp == midVal.messageStartTime())
            return mid;

         if (timestamp > midVal.messageStartTime())
         {
            if (timestamp <= midVal.messageEndTime())
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

   private static class ChunkExtra
   {
      private int index = -1;
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
            List<MCAP.Message> messages = bundledMessages.get(message.logTime());
            if (messages == null)
            {
               messages = new ArrayList<>();
               bundledMessages.put(message.logTime(), messages);
            }
            messages.add(message);
         }
      }
   }
}
