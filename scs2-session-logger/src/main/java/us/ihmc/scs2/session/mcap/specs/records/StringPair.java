package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.Objects;

public class StringPair implements MCAPElement
{
   private final String key;
   private final String value;

   public StringPair(MCAPDataInput dataInput, long elementPosition)
   {
      dataInput.position(elementPosition);
      key = dataInput.getString();
      value = dataInput.getString();
   }

   public StringPair(String key, String value)
   {
      this.key = key;
      this.value = value;
   }

   @Override
   public long getElementLength()
   {
      return key.length() + value.length() + 2 * Integer.BYTES;
   }

   public String key()
   {
      return key;
   }

   public String value()
   {
      return value;
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putString(key);
      dataOutput.putString(value);
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addString(key);
      crc32.addString(value);
      return crc32;
   }

   @Override
   public String toString()
   {
      return (key + ": " + value).replace("\n", "");
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof StringPair other && equals(other);
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof StringPair other)
      {
         if (!Objects.equals(key(), other.key()))
            return false;

         return Objects.equals(value(), other.value());
      }

      return false;
   }
}
