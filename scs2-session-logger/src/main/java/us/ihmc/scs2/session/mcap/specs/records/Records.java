package us.ihmc.scs2.session.mcap.specs.records;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;

import static us.ihmc.scs2.session.mcap.specs.records.MCAPElement.indent;

/**
 * @see <a href="https://mcap.dev/spec#records">MCAP Records</a>
 */
public class Records extends ArrayList<Record>
{
   public Records()
   {
   }

   public Records(Collection<? extends Record> collection)
   {
      super(collection);
   }

   public static Records load(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      Records records = new Records();
      MCAP.parseList(dataInput, RecordDataInputBacked::new, elementPosition, elementLength, records);
      return records;
   }

   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();

      for (Record record : this)
         record.updateCRC(crc32);

      return crc32;
   }

   public Records crop(long startTimestamp, long endTimestamp)
   {
      Records croppedRecords = new Records();

      for (Record record : this)
      {
         switch (record.op())
         {
            case HEADER:
            case FOOTER:
            case SCHEMA:
            case CHANNEL:
            case METADATA:
            case METADATA_INDEX:
            {
               croppedRecords.add(record);
               break;
            }
            case MESSAGE:
            {
               Message message = record.body();
               if (message.logTime() >= startTimestamp && message.logTime() <= endTimestamp)
                  croppedRecords.add(record);
               break;
            }
            case ATTACHMENT:
            {
               Attachment attachment = record.body();
               if (attachment.logTime() >= startTimestamp && attachment.logTime() <= endTimestamp)
                  croppedRecords.add(record);
               break;
            }
            default:
               throw new IllegalArgumentException("Unexpected value: " + record.op());
         }
      }
      return croppedRecords;
   }

   public long getElementLength()
   {
      // TODO Improve this by keeping track of modifications to the records.
      return stream().mapToLong(Record::getElementLength).sum();
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   public String toString(int indent)
   {
      if (isEmpty())
         return indent(getClass().getSimpleName() + ": []", indent);

      String out = getClass().getSimpleName() + "[\n";
      out += EuclidCoreIOTools.getCollectionString("\n", this, r -> r.toString(indent + 1));
      return indent(out, indent);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;

      if (object instanceof Records other)
      {
         if (size() != other.size())
            return false;
         for (int i = 0; i < size(); i++)
         {
            Record record = get(i);
            Record otherRecord = other.get(i);
            if (!record.equals(otherRecord))
               return false;
         }
         return true;
      }

      return false;
   }

   public List<MessageIndex> generateMessageIndexList()
   {
      TIntObjectHashMap<MutableMessageIndex> messageIndexMap = new TIntObjectHashMap<>();
      long messageIndexOffset = 0;

      for (Record record : this)
      {
         if (record.op() == Opcode.MESSAGE)
         {
            Message message = record.body();
            int channelId = message.channelId();

            MutableMessageIndex messageIndex = messageIndexMap.get(channelId);

            if (messageIndex == null)
            {
               messageIndex = new MutableMessageIndex();
               messageIndex.setChannelId(channelId);
               messageIndexMap.put(channelId, messageIndex);
            }

            MessageIndexEntry messageIndexEntry = new MessageIndexEntry(message.logTime(), messageIndexOffset);
            messageIndex.addMessageIndexEntry(messageIndexEntry);
         }
         messageIndexOffset += record.getElementLength();
      }

      List<MessageIndex> messageIndices = Arrays.asList(messageIndexMap.values(new MutableMessageIndex[messageIndexMap.size()]));
      messageIndices.sort(Comparator.comparingInt(MessageIndex::channelId));
      return messageIndices;
   }

   public static List<MessageIndexOffset> generateMessageIndexOffsets(long offset, List<? extends Record> messageIndexRecordList)
   {
      List<MessageIndexOffset> messageIndexOffsets = new ArrayList<>();

      long messageIndexOffset = offset;

      for (Record messageIndexRecord : messageIndexRecordList)
      {
         if (messageIndexRecord.op() != Opcode.MESSAGE_INDEX)
            throw new IllegalArgumentException("Expected a message index record, but got: " + messageIndexRecord.op());
         MessageIndex messageIndex = messageIndexRecord.body();
         messageIndexOffsets.add(new MessageIndexOffset(messageIndex.channelId(), messageIndexOffset));
         messageIndexOffset += messageIndexRecord.getElementLength();
      }

      return messageIndexOffsets;
   }
}
