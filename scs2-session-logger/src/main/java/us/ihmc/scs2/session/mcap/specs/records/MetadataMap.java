package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.LinkedHashMap;
import java.util.Map.Entry;

import static us.ihmc.scs2.session.mcap.specs.records.MCAPElement.stringLength;

public class MetadataMap extends LinkedHashMap<String, String> implements MCAPElement
{

   public MetadataMap()
   {
   }

   public MetadataMap(MCAPDataInput dataInput, long elementPosition)
   {
      dataInput.position(elementPosition);
      long length = dataInput.getUnsignedInt();

      while (dataInput.position() - elementPosition < length)
      {
         String key = dataInput.getString();
         String value = dataInput.getString();
         put(key, value);
      }
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putUnsignedInt(getElementLength());

      for (Entry<String, String> entry : entrySet())
      {
         dataOutput.putString(entry.getKey());
         dataOutput.putString(entry.getValue());
      }
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();

      crc32.addUnsignedInt(getElementLength());

      for (Entry<String, String> entry : entrySet())
      {
         crc32.addString(entry.getKey());
         crc32.addString(entry.getValue());
      }
      return crc32;
   }

   @Override
   public long getElementLength()
   {
      long length = Integer.BYTES;
      for (Entry<String, String> entry : entrySet())
      {
         length += stringLength(entry.getKey()) + stringLength(entry.getValue());
      }
      return length;
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      return mcapElement instanceof MetadataMap other && equals(other);
   }
}
