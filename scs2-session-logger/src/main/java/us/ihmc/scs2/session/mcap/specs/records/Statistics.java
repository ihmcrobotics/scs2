package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.List;
import java.util.Objects;

/**
 * A Statistics record contains summary information about the recorded data.
 * The statistics record is optional, but the file should contain at most one.
 *
 * @see <a href="https://mcap.dev/spec#statistics-op0x0b">MCAP Statistics</a>
 */
public interface Statistics extends MCAPElement
{
   static Statistics load(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      return new StatisticsDataInputBacked(dataInput, elementPosition, elementLength);
   }

   long messageCount();

   int schemaCount();

   long channelCount();

   long attachmentCount();

   long metadataCount();

   long chunkCount();

   long messageStartTime();

   long messageEndTime();

   List<ChannelMessageCount> channelMessageCounts();

   @Override
   default void write(MCAPDataOutput dataOutput)
   {
      dataOutput.putLong(messageCount());
      dataOutput.putUnsignedShort(schemaCount());
      dataOutput.putUnsignedInt(channelCount());
      dataOutput.putUnsignedInt(attachmentCount());
      dataOutput.putUnsignedInt(metadataCount());
      dataOutput.putUnsignedInt(chunkCount());
      dataOutput.putLong(messageStartTime());
      dataOutput.putLong(messageEndTime());
      dataOutput.putCollection(channelMessageCounts());
   }

   @Override
   default MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addLong(messageCount());
      crc32.addUnsignedShort(schemaCount());
      crc32.addUnsignedInt(channelCount());
      crc32.addUnsignedInt(attachmentCount());
      crc32.addUnsignedInt(metadataCount());
      crc32.addUnsignedInt(chunkCount());
      crc32.addLong(messageStartTime());
      crc32.addLong(messageEndTime());
      crc32.addCollection(channelMessageCounts());
      return crc32;
   }

   @Override
   default String toString(int indent)
   {
      String out = getClass().getSimpleName() + ": ";
      out += "\n\t-messageCount = " + messageCount();
      out += "\n\t-schemaCount = " + schemaCount();
      out += "\n\t-channelCount = " + channelCount();
      out += "\n\t-attachmentCount = " + attachmentCount();
      out += "\n\t-metadataCount = " + metadataCount();
      out += "\n\t-chunkCount = " + chunkCount();
      out += "\n\t-messageStartTime = " + messageStartTime();
      out += "\n\t-messageEndTime = " + messageEndTime();
      out += "\n\t-channelMessageCounts = \n" + EuclidCoreIOTools.getCollectionString("\n", channelMessageCounts(), e -> e.toString(1));
      return out;
   }

   @Override
   default boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof Statistics other)
      {
         if (messageCount() != other.messageCount())
            return false;
         if (schemaCount() != other.schemaCount())
            return false;
         if (channelCount() != other.channelCount())
            return false;
         if (attachmentCount() != other.attachmentCount())
            return false;
         if (metadataCount() != other.metadataCount())
            return false;
         if (chunkCount() != other.chunkCount())
            return false;
         if (messageStartTime() != other.messageStartTime())
            return false;
         if (messageEndTime() != other.messageEndTime())
            return false;
         return Objects.equals(channelMessageCounts(), other.channelMessageCounts());
      }

      return false;
   }
}
