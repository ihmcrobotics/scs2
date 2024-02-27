package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

public class MessageIndexEntry implements MCAPElement
{
   /**
    * Time at which the message was recorded.
    */
   private final long logTime;

   /**
    * Offset is relative to the start of the uncompressed chunk data.
    */
   private final long offset;

   /**
    * @param logTime time at which the message was recorded.
    * @param offset  offset is relative to the start of the uncompressed chunk data.
    */
   public MessageIndexEntry(long logTime, long offset)
   {
      this.logTime = MCAP.checkPositiveLong(logTime, "logTime");
      this.offset = MCAP.checkPositiveLong(offset, "offset");
   }

   public MessageIndexEntry(MCAPDataInput dataInput, long elementPosition)
   {
      dataInput.position(elementPosition);
      logTime = MCAP.checkPositiveLong(dataInput.getLong(), "logTime");
      offset = MCAP.checkPositiveLong(dataInput.getLong(), "offset");
   }

   @Override
   public long getElementLength()
   {
      return 2 * Long.BYTES;
   }

   public long logTime()
   {
      return logTime;
   }

   public long offset()
   {
      return offset;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putLong(logTime());
      dataOutput.putLong(offset());
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addLong(logTime());
      crc32.addLong(offset());
      return crc32;
   }

   @Override
   public String toString(int indent)
   {
      String out = getClass().getSimpleName() + ":";
      out += "\n\t-logTime = " + logTime();
      out += "\n\t-offset = " + offset();
      return MCAPElement.indent(out, indent);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof MessageIndexEntry other && equals(other);
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof MessageIndexEntry other)
      {
         if (logTime() != other.logTime())
            return false;
         return offset() == other.offset();
      }

      return false;
   }
}
