package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

public class MutableStatistics implements Statistics
{
   private long messageCount;
   private int schemaCount;
   private long channelCount;
   private long attachmentCount;
   private long metadataCount;
   private long chunkCount;
   private long messageStartTime;
   private long messageEndTime;
   private List<ChannelMessageCount> channelMessageCounts; // The list index is the channel index
   private long channelMessageCountsLength;

   @Override
   public long getElementLength()
   {
      return 3 * Long.BYTES + 5 * Integer.BYTES + Short.BYTES + channelMessageCountsLength;
   }

   public void incrementCount(Opcode op)
   {
      switch (op)
      {
         case MESSAGE:
            incrementMessageCount();
            break;
         case SCHEMA:
            incrementSchemaCount();
            break;
         case CHANNEL:
            incrementChannelCount();
            break;
         case ATTACHMENT:
            incrementAttachmentCount();
            break;
         case METADATA:
            incrementMetadataCount();
            break;
         case CHUNK:
            incrementChunkCount();
            break;
         default:
            // Do nothing
      }
   }

   public void incrementMessageCount()
   {
      messageCount++;
   }

   public void setMessageCount(long messageCount)
   {
      this.messageCount = messageCount;
   }

   public void incrementSchemaCount()
   {
      schemaCount++;
   }

   public void setSchemaCount(int schemaCount)
   {
      this.schemaCount = schemaCount;
   }

   public void incrementChannelCount()
   {
      channelCount++;
   }

   public void setChannelCount(long channelCount)
   {
      this.channelCount = channelCount;
   }

   public void incrementAttachmentCount()
   {
      attachmentCount++;
   }

   public void setAttachmentCount(long attachmentCount)
   {
      this.attachmentCount = attachmentCount;
   }

   public void incrementMetadataCount()
   {
      metadataCount++;
   }

   public void setMetadataCount(long metadataCount)
   {
      this.metadataCount = metadataCount;
   }

   public void incrementChunkCount()
   {
      chunkCount++;
   }

   public void setChunkCount(long chunkCount)
   {
      this.chunkCount = chunkCount;
   }

   public void updateMessageTimes(long messageStartTime, long messageEndTime)
   {
      updateMessageStartTime(messageStartTime);
      updateMessageEndTime(messageEndTime);
   }

   public void updateMessageStartTime(long messageStartTime)
   {
      if (this.messageStartTime == 0)
         this.messageStartTime = messageStartTime;
      else
         this.messageStartTime = Math.min(this.messageStartTime, messageStartTime);
   }

   public void setMessageStartTime(long messageStartTime)
   {
      this.messageStartTime = messageStartTime;
   }

   public void updateMessageEndTime(long messageEndTime)
   {
      if (this.messageEndTime == 0)
         this.messageEndTime = messageEndTime;
      else
         this.messageEndTime = Math.max(this.messageEndTime, messageEndTime);
   }

   public void setMessageEndTime(long messageEndTime)
   {
      this.messageEndTime = messageEndTime;
   }

   public void incrementChannelMessageCount(int channelIndex)
   {
      if (channelMessageCounts == null)
         channelMessageCounts = new ArrayList<>();

      while (channelMessageCounts.size() <= channelIndex)
         channelMessageCounts.add(null);

      ChannelMessageCount count = channelMessageCounts.get(channelIndex);

      if (count == null)
      {
         count = new ChannelMessageCount(channelIndex, 1);
         channelMessageCounts.set(channelIndex, count);
         channelMessageCountsLength += ChannelMessageCount.ELEMENT_LENGTH;
      }
      else
      {
         count.incrementMessageCount();
      }
   }

   public void setChannelMessageCounts(List<ChannelMessageCount> channelMessageCounts)
   {
      this.channelMessageCounts = channelMessageCounts;
      channelMessageCountsLength = channelMessageCounts.size() * ChannelMessageCount.ELEMENT_LENGTH;
   }

   @Override
   public long messageCount()
   {
      return messageCount;
   }

   @Override
   public int schemaCount()
   {
      return schemaCount;
   }

   @Override
   public long channelCount()
   {
      return channelCount;
   }

   @Override
   public long attachmentCount()
   {
      return attachmentCount;
   }

   @Override
   public long metadataCount()
   {
      return metadataCount;
   }

   @Override
   public long chunkCount()
   {
      return chunkCount;
   }

   @Override
   public long messageStartTime()
   {
      return messageStartTime;
   }

   @Override
   public long messageEndTime()
   {
      return messageEndTime;
   }

   @Override
   public List<ChannelMessageCount> channelMessageCounts()
   {
      return channelMessageCounts;
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putLong(messageCount());
      dataOutput.putUnsignedShort(schemaCount());
      dataOutput.putUnsignedInt(channelCount());
      dataOutput.putUnsignedInt(attachmentCount());
      dataOutput.putUnsignedInt(metadataCount());
      dataOutput.putUnsignedInt(chunkCount());
      dataOutput.putLong(messageStartTime());
      dataOutput.putLong(messageEndTime());
      if (channelMessageCounts() == null)
         dataOutput.putCollection(Collections.emptyList());
      else
         dataOutput.putCollection(channelMessageCounts().stream().filter(Objects::nonNull).toList());
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addLong(messageCount());
      crc32.addUnsignedShort(schemaCount());
      crc32.addUnsignedInt(channelCount());
      crc32.addUnsignedInt(attachmentCount());
      crc32.addUnsignedInt(metadataCount());
      crc32.addUnsignedInt(chunkCount());
      crc32.addLong(messageStartTime());
      crc32.addLong(messageEndTime());
      if (channelMessageCounts() == null)
         crc32.addCollection(Collections.emptyList());
      else
         crc32.addCollection(channelMessageCounts().stream().filter(Objects::nonNull).toList());
      return crc32;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Statistics other && Statistics.super.equals(other);
   }
}
