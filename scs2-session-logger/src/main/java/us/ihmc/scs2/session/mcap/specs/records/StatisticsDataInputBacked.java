package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.util.List;

public class StatisticsDataInputBacked implements Statistics
{
   private final MCAPDataInput dataInput;
   private final long elementLength;
   private final long messageCount;
   private final int schemaCount;
   private final long channelCount;
   private final long attachmentCount;
   private final long metadataCount;
   private final long chunkCount;
   private final long messageStartTime;
   private final long messageEndTime;
   private WeakReference<List<ChannelMessageCount>> channelMessageCountsRef;
   private final long channelMessageCountsOffset;
   private final long channelMessageCountsLength;

   public StatisticsDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;
      this.elementLength = elementLength;

      dataInput.position(elementPosition);
      messageCount = MCAP.checkPositiveLong(dataInput.getLong(), "messageCount");
      schemaCount = dataInput.getUnsignedShort();
      channelCount = dataInput.getUnsignedInt();
      attachmentCount = dataInput.getUnsignedInt();
      metadataCount = dataInput.getUnsignedInt();
      chunkCount = dataInput.getUnsignedInt();
      messageStartTime = MCAP.checkPositiveLong(dataInput.getLong(), "messageStartTime");
      messageEndTime = MCAP.checkPositiveLong(dataInput.getLong(), "messageEndTime");
      channelMessageCountsLength = dataInput.getUnsignedInt();
      channelMessageCountsOffset = dataInput.position();
   }

   @Override
   public long getElementLength()
   {
      return elementLength;
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
      List<ChannelMessageCount> channelMessageCounts = channelMessageCountsRef == null ? null : channelMessageCountsRef.get();

      if (channelMessageCounts == null)
      {
         channelMessageCounts = MCAP.parseList(dataInput, ChannelMessageCount::new, channelMessageCountsOffset, channelMessageCountsLength);
         channelMessageCountsRef = new WeakReference<>(channelMessageCounts);
      }

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
