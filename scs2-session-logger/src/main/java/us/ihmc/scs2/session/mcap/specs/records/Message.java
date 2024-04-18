package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.nio.ByteBuffer;
import java.util.Objects;

/**
 * Message records encode a single timestamped message on a channel.
 * The message encoding and schema must match that of the Channel record corresponding to the message's channel ID.
 *
 * @see <a href="https://mcap.dev/spec#message-op0x05">MCAP Message</a>
 */
public interface Message extends MCAPElement
{
   static Message load(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      return new MessageDataInputBacked(dataInput, elementPosition, elementLength);
   }

   int channelId();

   long sequence();

   long logTime();

   long publishTime();

   int dataLength();

   ByteBuffer messageBuffer();

   byte[] messageData();

   @Override
   default void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putUnsignedShort(channelId());
      dataOutput.putUnsignedInt(sequence());
      dataOutput.putLong(logTime());
      dataOutput.putLong(publishTime());
      dataOutput.putByteBuffer(messageBuffer());
   }

   @Override
   default MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addUnsignedShort(channelId());
      crc32.addUnsignedInt(sequence());
      crc32.addLong(logTime());
      crc32.addLong(publishTime());
      crc32.addByteBuffer(messageBuffer());
      return crc32;
   }

   @Override
   default long getElementLength()
   {
      return dataLength() + Short.BYTES + Integer.BYTES + 2 * Long.BYTES;
   }

   @Override
   default String toString(int indent)
   {
      String out = getClass().getSimpleName() + ": ";
      out += "\n\t-channelId = " + channelId();
      out += "\n\t-sequence = " + sequence();
      out += "\n\t-logTime = " + logTime();
      out += "\n\t-publishTime = " + publishTime();
      //         out += "\n\t-data = " + Arrays.toString(messageData());
      return MCAPElement.indent(out, indent);
   }

   @Override
   default boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof Message other)
      {
         if (channelId() != other.channelId())
            return false;
         if (sequence() != other.sequence())
            return false;
         if (logTime() != other.logTime())
            return false;
         if (publishTime() != other.publishTime())
            return false;
         if (dataLength() != other.dataLength())
            return false;
         return Objects.equals(messageBuffer(), other.messageBuffer());
      }

      return false;
   }
}
