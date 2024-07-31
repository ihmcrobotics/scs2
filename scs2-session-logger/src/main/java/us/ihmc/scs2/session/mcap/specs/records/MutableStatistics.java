package us.ihmc.scs2.session.mcap.specs.records;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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
   private List<ChannelMessageCount> channelMessageCounts;
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
      Optional<ChannelMessageCount> countOptional = channelMessageCounts.stream().filter(count -> count.channelId() == channelIndex).findFirst();

      if (countOptional.isPresent())
      {
         countOptional.get().incrementMessageCount();
      }
      else
      {
         channelMessageCounts.add(new ChannelMessageCount(channelIndex, 1));
         channelMessageCountsLength += ChannelMessageCount.ELEMENT_LENGTH;
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
