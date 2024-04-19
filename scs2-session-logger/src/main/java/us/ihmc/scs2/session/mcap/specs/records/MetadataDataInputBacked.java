package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.util.List;

/**
 * A metadata record contains arbitrary user data in key-value pairs.
 *
 * @see <a href="https://mcap.dev/spec#metadata-op0x0c">MCAP Metadata</a>
 */
public class MetadataDataInputBacked implements Metadata
{
   private final String name;
   private final List<StringPair> metadata;
   private final int metadataLength;

   MetadataDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      dataInput.position(elementPosition);
      name = dataInput.getString();
      long start = dataInput.position();
      metadata = MCAP.parseList(dataInput, StringPair::new); // TODO Looks into postponing the loading of the metadata.
      metadataLength = (int) (dataInput.position() - start);
      MCAP.checkLength(elementLength, getElementLength());
   }

   @Override
   public long getElementLength()
   {
      return Integer.BYTES + name.length() + metadataLength;
   }

   @Override
   public String name()
   {
      return name;
   }

   @Override
   public List<StringPair> metadata()
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
