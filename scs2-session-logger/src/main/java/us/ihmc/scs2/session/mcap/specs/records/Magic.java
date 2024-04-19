package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.Arrays;

/**
 * @see <a href="https://mcap.dev/spec#magic">MCAP Magic</a>
 */
public class Magic implements MCAPElement
{
   public static final int MAGIC_SIZE = 8;
   public static final byte[] MAGIC_BYTES = {-119, 77, 67, 65, 80, 48, 13, 10};

   public static final Magic INSTANCE = new Magic();

   private Magic()
   {

   }

   public static void readMagic(MCAPDataInput dataInput, long elementPosition)
   {
      dataInput.position(elementPosition);
      byte[] magic = dataInput.getBytes(MAGIC_SIZE);
      if (!(Arrays.equals(magic, MAGIC_BYTES)))
         throw new IllegalArgumentException("Invalid magic bytes: " + Arrays.toString(magic) + ". Expected: " + Arrays.toString(MAGIC_BYTES));
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      // Magic bytes are not included in any CRC calculation
      return crc32;
   }

   @Override
   public void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putBytes(MAGIC_BYTES);
   }

   @Override
   public long getElementLength()
   {
      return MAGIC_SIZE;
   }

   @Override
   public String toString()
   {
      return "Magic: " + Arrays.toString(MAGIC_BYTES);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Magic;
   }

   @Override
   public boolean equals(MCAPElement mcapElement)
   {
      return mcapElement instanceof Magic;
   }
}
