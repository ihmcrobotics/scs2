package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.Arrays;

/**
 * @see <a href="https://mcap.dev/spec#magic">MCAP Magic</a>
 */
public class Magic
{
   public static final int MAGIC_SIZE = 8;
   public static final byte[] MAGIC_BYTES = {-119, 77, 67, 65, 80, 48, 13, 10};

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

   public static void writeMagic(MCAPDataOutput dataOutput)
   {
      dataOutput.putBytes(MAGIC_BYTES);
   }

   public static long getElementLength()
   {
      return MAGIC_SIZE;
   }
}
