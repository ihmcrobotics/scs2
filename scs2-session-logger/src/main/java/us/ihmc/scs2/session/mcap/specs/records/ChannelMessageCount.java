package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

public class ChannelMessageCount implements MCAPElement
{
   public static final long ELEMENT_LENGTH = Short.BYTES + Long.BYTES;
   private final int channelId;
   private long messageCount;

   public ChannelMessageCount(MCAPDataInput dataInput, long elementPosition)
   {
      dataInput.position(elementPosition);
      channelId = dataInput.getUnsignedShort();
      messageCount = dataInput.getLong();
   }

   public ChannelMessageCount(int channelId)
   {
      this(channelId, 0);
   }

   public ChannelMessageCount(int channelId, long messageCount)
   {
      this.channelId = channelId;
      this.messageCount = messageCount;
   }

   @Override
   public long getElementLength()
   {
      return ELEMENT_LENGTH;
   }

   public int channelId()
   {
      return channelId;
   }

   public void incrementMessageCount()
   {
      messageCount++;
   }

   public void setMessageCount(long messageCount)
   {
      this.messageCount = messageCount;
   }

   public long messageCount()
   {
      return messageCount;
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putUnsignedShort(channelId);
      dataOutput.putLong(messageCount);
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addUnsignedShort(channelId);
      crc32.addLong(messageCount);
      return crc32;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public String toString(int indent)
   {
      String out = getClass().getSimpleName() + ":";
      out += "\n\t-channelId = " + channelId;
      out += "\n\t-messageCount = " + messageCount;
      return MCAPElement.indent(out, indent);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof ChannelMessageCount other && equals(other);
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof ChannelMessageCount other)
      {
         if (channelId() != other.channelId())
            return false;
         return messageCount() == other.messageCount();
      }

      return false;
   }
}
