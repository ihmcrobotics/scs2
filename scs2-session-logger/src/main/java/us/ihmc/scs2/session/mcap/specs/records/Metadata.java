package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.Objects;

public interface Metadata extends MCAPElement
{
   String name();

   MetadataMap metadata();

   @Override
   default long getElementLength()
   {
      return MCAPElement.stringLength(name()) + metadata().getElementLength();
   }

   @Override
   default void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putString(name());
      metadata().write(dataOutput);
   }

   @Override
   default MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addString(name());
      metadata().updateCRC(crc32);
      return crc32;
   }

   @Override
   default String toString(int indent)
   {
      String out = getClass().getSimpleName() + ": ";
      out += "\n\t-name = " + name();
      out += "\n\t-metadata = " + EuclidCoreIOTools.getCollectionString(", ", metadata().keySet());
      return MCAPElement.indent(out, indent);
   }

   @Override
   default boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof Metadata other)
      {
         if (!name().equals(other.name()))
            return false;
         return Objects.equals(metadata(), other.metadata());
      }

      return false;
   }
}
