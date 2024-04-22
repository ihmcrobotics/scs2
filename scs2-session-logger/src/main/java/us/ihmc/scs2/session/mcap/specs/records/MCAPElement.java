package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

public interface MCAPElement
{
   static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = "\t".repeat(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
   }

   void write(MCAPDataOutput dataOutput);

   /**
    * Update the CRC32 with the data of this element.
    * <p>
    * Note that this method only the given {@code crc32}, it does not modify this element.
    * </p>
    *
    * @param crc32 the CRC32 to update.
    * @return the updated CRC32 or a new instance of {@code MCAPCRC32Helper} if {@code crc32} is {@code null}.
    */
   MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32);

   long getElementLength();

   default String toString(int indent)
   {
      return indent(toString(), indent);
   }

   boolean equals(MCAPElement mcapElement);

   static long stringLength(String string)
   {
      return Integer.BYTES + string.length();
   }
}
