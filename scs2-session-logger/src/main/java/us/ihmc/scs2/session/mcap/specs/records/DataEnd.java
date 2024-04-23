package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

/**
 * A Data End record indicates the end of the data section.
 *
 * @see <a href="https://mcap.dev/spec#data-end-op0x0f">MCAP Data End</a>
 */
public class DataEnd implements MCAPElement
{
   private long dataSectionCRC32;

   public DataEnd(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      dataInput.position(elementPosition);
      dataSectionCRC32 = dataInput.getUnsignedInt();
      MCAP.checkLength(elementLength, getElementLength());
   }

   public DataEnd(long dataSectionCRC32)
   {
      this.dataSectionCRC32 = dataSectionCRC32;
   }

   public void setDataSectionCRC32(long dataSectionCRC32)
   {
      this.dataSectionCRC32 = dataSectionCRC32;
   }

   @Override
   public long getElementLength()
   {
      return Integer.BYTES;
   }

   /**
    * CRC-32 of all bytes in the data section. A value of 0 indicates the CRC-32 is not available.
    */
   public long dataSectionCRC32()
   {
      return dataSectionCRC32;
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putUnsignedInt(dataSectionCRC32);
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addUnsignedInt(dataSectionCRC32);
      return crc32;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ":\n\t-dataSectionCrc32 = " + dataSectionCRC32;
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof DataEnd other && equals(other);
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof DataEnd other)
         return dataSectionCRC32() == other.dataSectionCRC32();

      return false;
   }
}
