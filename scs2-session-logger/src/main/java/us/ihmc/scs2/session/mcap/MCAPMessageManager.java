package us.ihmc.scs2.session.mcap;

import gnu.trove.list.array.TLongArrayList;
import us.ihmc.scs2.session.mcap.MCAPBufferedChunk.ChunkBundle;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.ChunkIndex;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.MessageIndex;
import us.ihmc.scs2.session.mcap.specs.records.MessageIndexEntry;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * This class is used to simplify retrieval of messages from an MCAP file.
 */
public class MCAPMessageManager
{
   /**
    * The timestamps of all the messages in the MCAP file.
    */
   private final TLongArrayList allMessageTimestamps = new TLongArrayList();
   /**
    * All the chunk indices of the MCAP file.
    */
   private final List<ChunkIndex> mcapChunkIndices = new ArrayList<>();
   private final MCAPBufferedChunk chunkBuffer;
   private ChunkBundle currentChunkBundle = null;
   private final long desiredLogDT;

   public MCAPMessageManager(MCAP mcap, MCAPBufferedChunk chunkBuffer, long desiredLogDT)
   {
      this.chunkBuffer = chunkBuffer;
      this.desiredLogDT = desiredLogDT;
      loadFromMCAP(mcap);
   }

   /**
    * @return the instant of the message at the given index.
    */
   public long getTimestampAtIndex(int index)
   {
      return allMessageTimestamps.get(index);
   }

   /**
    * @return the instant of the message at the given index relative to the first message instant.
    */
   public long getRelativeTimestampAtIndex(int index)
   {
      return allMessageTimestamps.get(index) - firstMessageTimestamp();
   }

   /**
    * @return the index of the message with the given instant.
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
      for (Record record : mcap.records())
      {
         if (record.op() == Opcode.CHUNK_INDEX)
         {
            mcapChunkIndices.add((ChunkIndex) record.body());
         }
         else if (record.op() == Opcode.MESSAGE_INDEX)
         {
            MessageIndex messageIndex = (MessageIndex) record.body();
            for (MessageIndexEntry mcapEntry : messageIndex.messageIndexEntries())
            {
               long timestamp = round(mcapEntry.logTime(), desiredLogDT);
               if (allMessageTimestamps.isEmpty() || timestamp > allMessageTimestamps.get(allMessageTimestamps.size() - 1))
               {
                  allMessageTimestamps.add(timestamp);
               }
               else if (timestamp == allMessageTimestamps.get(allMessageTimestamps.size() - 1))
               {
                  // Nothing, the instant is already there.
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
                     // Nothing, the instant is already there.
                  }
               }
            }
         }
      }
      // TODO The underlying algorithm seems quite expensive.
      mcapChunkIndices.sort(Comparator.comparingLong(chunkIndex -> round(chunkIndex.messageStartTime(), desiredLogDT)));
   }

   /**
    * @return the instant of the first message in the MCAP file.
    */
   public long firstMessageTimestamp()
   {
      return round(mcapChunkIndices.get(0).messageStartTime(), desiredLogDT);
   }

   /**
    * @return the instant of the last message in the MCAP file.
    */
   public long lastMessageTimestamp()
   {
      return round(mcapChunkIndices.get(mcapChunkIndices.size() - 1).messageEndTime(), desiredLogDT);
   }

   /**
    * @return the instant of the next message after the given instant.
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
    * @return retrieves the messages at the given instant.
    */
   public List<Message> loadMessages(long timestamp) throws IOException
   {
      if (currentChunkBundle == null || timestamp < currentChunkBundle.startTime() || timestamp > currentChunkBundle.endTime())
      {
         currentChunkBundle = chunkBuffer.getChunkBundle(timestamp);
         if (currentChunkBundle == null)
            return Collections.emptyList();
         currentChunkBundle.requestLoadChunkBundle(true);
      }

      if (currentChunkBundle.startTime() == timestamp)
      {
         ChunkBundle previous = currentChunkBundle.previous();
         if (previous == null)
            return currentChunkBundle.getMessages(timestamp);

         previous.requestLoadChunkBundle(true);
         return merge(previous.getMessages(timestamp), currentChunkBundle.getMessages(timestamp));
      }
      else if (currentChunkBundle.endTime() == timestamp)
      {
         ChunkBundle next = currentChunkBundle.next();
         if (next == null)
            return currentChunkBundle.getMessages(timestamp);

         next.requestLoadChunkBundle(true);
         return merge(currentChunkBundle.getMessages(timestamp), next.getMessages(timestamp));
      }
      else
      {
         return currentChunkBundle.getMessages(timestamp);
      }
   }

   private static <T> List<T> merge(List<T> listA, List<T> listB)
   {
      if (listA == null && listB == null)
         return Collections.emptyList();
      if (listA == null)
         return listB;
      if (listB == null)
         return listA;

      List<T> merged = new ArrayList<>(listA.size() + listB.size());
      merged.addAll(listA);
      merged.addAll(listB);
      return merged;
   }

   /**
    * @return the instant of the first message in the currently loaded chunk.
    */
   public long getActiveChunkStartTimestamp()
   {
      if (currentChunkBundle == null)
         return -1;
      else
         return currentChunkBundle.startTime();
   }

   /**
    * @return the instant of the last message in the currently loaded chunk.
    */
   public long getActiveChunkEndTimestamp()
   {
      if (currentChunkBundle == null)
         return -1;
      else
         return currentChunkBundle.endTime();
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
}
