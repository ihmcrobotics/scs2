package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import static us.ihmc.scs2.session.mcap.specs.records.MCAPElement.stringLength;

/**
 * A metadata record contains arbitrary user data in key-value pairs.
 *
 * @see <a href="https://mcap.dev/spec#metadata-op0x0c">MCAP Metadata</a>
 */
public class MetadataDataInputBacked implements Metadata
{
   private final String name;
   private final MetadataMap metadata;
   private final int metadataLength;

   MetadataDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      dataInput.position(elementPosition);
      name = dataInput.getString();
      long start = dataInput.position();
      metadata = new MetadataMap(dataInput, start);
      metadataLength = (int) (dataInput.position() - start);
      MCAP.checkLength(elementLength, getElementLength());
   }

   @Override
   public long getElementLength()
   {
      return stringLength(name) + metadataLength;
   }

   @Override
   public String name()
   {
      return name;
   }

   @Override
   public MetadataMap metadata()
   {
      return metadata;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Metadata other && equals(other);
   }
}
