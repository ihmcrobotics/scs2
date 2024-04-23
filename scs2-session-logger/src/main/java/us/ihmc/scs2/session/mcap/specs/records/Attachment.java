package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.Arrays;
import java.util.Objects;

import static us.ihmc.scs2.session.mcap.specs.records.MCAPElement.stringLength;

/**
 * Attachment records contain auxiliary artifacts such as text, core dumps, calibration data, or other arbitrary data.
 * Attachment records must not appear within a chunk.
 *
 * @see <a href="https://mcap.dev/spec#attachment-op0x09">MCAP Attachment</a>
 */
public interface Attachment extends MCAPElement
{
   static Attachment load(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      return new AttachmentDataInputBacked(dataInput, elementPosition, elementLength);
   }

   @Override
   default long getElementLength()
   {
      return 3 * Long.BYTES + Integer.BYTES + stringLength(name()) + stringLength(mediaType()) + (int) dataLength();
   }

   /** Time at which the attachment was recorded. */
   long logTime();

   /** Time at which the attachment was created. If not available, must be set to zero. */
   long createTime();

   /** Name of the attachment, e.g "scene1.jpg". */
   String name();

   /** <a href="https://en.wikipedia.org/wiki/Media_type">Media type</a> of the attachment (e.g "text/plain"). */
   String mediaType();

   /** Size in bytes of the attachment data. Typically equal to {@code this.data().length}. */
   long dataLength();

   /** Attachment data. */
   byte[] data();

   /**
    * CRC-32 checksum of the preceding fields in the record.
    * A value of zero indicates that CRC validation should not be performed.
    */
   long crc32();

   @Override
   default void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putLong(logTime());
      dataOutput.putLong(createTime());
      dataOutput.putString(name());
      dataOutput.putString(mediaType());
      dataOutput.putLong(dataLength());
      dataOutput.putBytes(data());
      dataOutput.putUnsignedInt(crc32());
   }

   @Override
   default MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addLong(logTime());
      crc32.addLong(createTime());
      crc32.addString(name());
      crc32.addString(mediaType());
      crc32.addLong(dataLength());
      crc32.addBytes(data());
      return crc32;
   }

   @Override
   default String toString(int indent)
   {
      String out = getClass().getSimpleName() + ": ";
      out += "\n\t-logTime = " + logTime();
      out += "\n\t-createTime = " + createTime();
      out += "\n\t-name = " + name();
      out += "\n\t-mediaType = " + mediaType();
      out += "\n\t-dataLength = " + dataLength();
      out += "\n\t-data = " + Arrays.toString(data());
      out += "\n\t-crc32 = " + crc32();
      return MCAPElement.indent(out, indent);
   }

   @Override
   default boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof Attachment other)
      {
         if (logTime() != other.logTime())
            return false;
         if (createTime() != other.createTime())
            return false;
         if (!Objects.equals(name(), other.name()))
            return false;
         if (!Objects.equals(mediaType(), other.mediaType()))
            return false;
         if (dataLength() != other.dataLength())
            return false;
         if (!Arrays.equals(data(), other.data()))
            return false;
         return crc32() == other.crc32();
      }

      return false;
   }
}
