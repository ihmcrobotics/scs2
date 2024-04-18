package us.ihmc.scs2.session.mcap.specs.records;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.util.AbstractList;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import java.util.function.Predicate;

import static us.ihmc.scs2.session.mcap.specs.records.MCAPElement.indent;

/**
 * @see <a href="https://mcap.dev/spec#records">MCAP Records</a>
 */
public class Records extends AbstractList<Record>
{
   private final List<Record> records = new ArrayList<>();
   private long messageStartTime = 0L;
   private long messageEndTime = 0L;
   private long crc32 = 0L;

   public Records()
   {
   }

   public Records(Collection<? extends Record> collection)
   {
      addAll(collection);
   }

   public static Records load(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      List<Record> records = new ArrayList<>();
      MCAP.parseList(dataInput, RecordDataInputBacked::new, elementPosition, elementLength, records);
      return new Records(records);
   }

   public void sortByTimestamp()
   {
      records.sort(Comparator.comparingLong(r ->
                                            {// schemas, then channels, and then messages in chronological order.
                                               if (r.op() == Opcode.MESSAGE)
                                                  return ((Message) r.body()).logTime();
                                               else if (r.op() == Opcode.ATTACHMENT)
                                                  return ((Attachment) r.body()).logTime();
                                               else if (r.op() == Opcode.CHANNEL)
                                                  return Long.MIN_VALUE + 1;
                                               else if (r.op() == Opcode.SCHEMA)
                                                  return Long.MIN_VALUE;
                                               else
                                                  return 0;
                                            }));
      updateInfo();
   }

   public boolean containsMessages()
   {
      return records.stream().anyMatch(r -> r.op() == Opcode.MESSAGE);
   }

   @Override
   public boolean add(Record record)
   {
      records.add(Objects.requireNonNull(record));
      updateInfo();
      return true;
   }

   @Override
   public void add(int index, Record element)
   {
      records.add(index, Objects.requireNonNull(element));
      updateInfo();
   }

   @Override
   public boolean addAll(Collection<? extends Record> c)
   {
      records.addAll(c);
      updateInfo();
      return true;
   }

   @Override
   public boolean addAll(int index, Collection<? extends Record> c)
   {
      records.addAll(index, c);
      updateInfo();
      return true;
   }

   @Override
   public Record set(int index, Record element)
   {
      Record old = records.set(index, element);
      updateInfo();
      return old;
   }

   @Override
   public Record remove(int index)
   {
      Record old = records.remove(index);
      updateInfo();
      return old;
   }

   @Override
   protected void removeRange(int fromIndex, int toIndex)
   {
      records.subList(fromIndex, toIndex).clear();
      updateInfo();
   }

   @Override
   public Record get(int index)
   {
      return records.get(index);
   }

   public int indexOf(Predicate<Record> predicate)
   {
      for (int i = 0; i < records.size(); i++)
      {
         if (predicate.test(records.get(i)))
            return i;
      }
      return -1;
   }

   @Override
   public int size()
   {
      return records.size();
   }

   private void updateInfo()
   {
      messageStartTime = stream().filter(r -> r.op() == Opcode.MESSAGE).mapToLong(r -> ((Message) r.body()).logTime()).min().orElse(0);
      messageEndTime = stream().filter(r -> r.op() == Opcode.MESSAGE).mapToLong(r -> ((Message) r.body()).logTime()).max().orElse(0);
      crc32 = new MCAPCRC32Helper().addHeadlessCollection(records).getValue();
   }

   public long getMessageStartTime()
   {
      return messageStartTime;
   }

   public long getMessageEndTime()
   {
      return messageEndTime;
   }

   public long getCRC32()
   {
      return crc32;
   }

   public Records crop(long startTimestamp, long endTimestamp)
   {
      List<Record> croppedRecords = new ArrayList<>();

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
      return new Records(croppedRecords);
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
