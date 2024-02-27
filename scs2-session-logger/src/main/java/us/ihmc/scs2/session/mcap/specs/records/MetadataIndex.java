package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.Objects;

/**
 * A metadata index record contains the location of a metadata record within the file.
 *
 * @see <a href="https://mcap.dev/spec#metadata-index-op0x0d">MCAP Metadata Index</a>
 */
public interface MetadataIndex extends MCAPElement
{
   @Override
   default long getElementLength()
   {
      return 2 * Long.BYTES + Integer.BYTES + name().length();
   }

   Record metadata();

   long metadataOffset();

   long metadataLength();

   String name();

   @Override
   default void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putLong(metadataOffset());
      dataOutput.putLong(metadataLength());
      dataOutput.putString(name());
   }

   @Override
   default MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addLong(metadataOffset());
      crc32.addLong(metadataLength());
      crc32.addString(name());
      return crc32;
   }

   @Override
   default String toString(int indent)
   {
      String out = getClass().getSimpleName() + ": ";
      out += "\n\t-metadataOffset = " + metadataOffset();
      out += "\n\t-metadataLength = " + metadataLength();
      out += "\n\t-name = " + name();
      return MCAPElement.indent(out, indent);
   }

   @Override
   default boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof MetadataIndex other)
      {
         if (metadataOffset() != other.metadataOffset())
            return false;
         if (metadataLength() != other.metadataLength())
            return false;
         if (!Objects.equals(name(), other.name()))
            return false;
         return Objects.equals(metadata(), other.metadata());
      }

      return false;
   }
}
