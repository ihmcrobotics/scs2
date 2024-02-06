package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput.Compression;

import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * MCAP is a modular container format and logging library for pub/sub messages with arbitrary
 * message serialization. It is primarily intended for use in robotics applications, and works well
 * under various workloads, resource constraints, and durability requirements. Time values
 * (`log_time`, `publish_time`, `create_time`) are represented in nanoseconds since a
 * user-understood epoch (i.e. Unix epoch, robot boot time, etc.)
 *
 * @see <a href="https://github.com/foxglove/mcap/tree/c1cc51d/docs/specification#readme">Source</a>
 */
public class MCAP
{
   /**
    * Stream object that this MCAP was parsed from.
    */
   protected MCAPDataInput dataInput;

   public enum Opcode
   {
      HEADER(1),
      FOOTER(2),
      SCHEMA(3),
      CHANNEL(4),
      MESSAGE(5),
      CHUNK(6),
      MESSAGE_INDEX(7),
      CHUNK_INDEX(8),
      ATTACHMENT(9),
      ATTACHMENT_INDEX(10),
      STATISTICS(11),
      METADATA(12),
      METADATA_INDEX(13),
      SUMMARY_OFFSET(14),
      DATA_END(15);

      private final long id;

      Opcode(long id)
      {
         this.id = id;
      }

      public long id()
      {
         return id;
      }

      private static final TLongObjectHashMap<Opcode> byId = new TLongObjectHashMap<>(15);

      static
      {
         for (Opcode e : Opcode.values())
            byId.put(e.id(), e);
      }

      public static Opcode byId(long id)
      {
         return byId.get(id);
      }
   }

   private final Magic headerMagic;
   private final List<Record> records;
   private final Magic footerMagic;

   private Record footer;

   public MCAP(FileChannel fileChannel)
   {
      dataInput = MCAPDataInput.wrap(fileChannel);

      long currentPos = 0;
      headerMagic = new Magic(dataInput, currentPos);
      currentPos += headerMagic.getElementLength();
      records = new ArrayList<>();
      Record lastRecord;

      do
      {
         lastRecord = new Record(dataInput, currentPos);
         if (lastRecord.getElementLength() < 0)
            throw new IllegalArgumentException("Invalid record length: " + lastRecord.getElementLength());
         currentPos += lastRecord.getElementLength();
         records.add(lastRecord);
      }
      while (!(lastRecord.op() == Opcode.FOOTER));

      footerMagic = new Magic(dataInput, currentPos);
   }

   public MCAPDataInput getDataInput()
   {
      return dataInput;
   }

   public Magic headerMagic()
   {
      return headerMagic;
   }

   public List<Record> records()
   {
      return records;
   }

   public Magic footerMagic()
   {
      return footerMagic;
   }

   public Record footer()
   {
      if (footer == null)
      {
         footer = new Record(dataInput, computeOffsetFooter(dataInput));
      }
      return footer;
   }

   public static class Chunk implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      /**
       * Earliest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private final long messageStartTime;
      /**
       * Latest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private final long messageEndTime;
      /**
       * Uncompressed size of the records field.
       */
      private final long recordsUncompressedLength;
      /**
       * CRC32 checksum of uncompressed records field. A value of zero indicates that CRC validation
       * should not be performed.
       */
      private final long uncompressedCrc32;
      /**
       * compression algorithm. i.e. zstd, lz4, "". An empty string indicates no compression. Refer to
       * well-known compression formats.
       */
      private final String compression;
      /**
       * Offset position of the records in either in the {@code  ByteBuffer} or {@code FileChannel},
       * depending on how this chunk was created.
       */
      private final long recordsOffset;
      /**
       * Length of the records in bytes.
       */
      private final long recordsCompressedLength;
      /**
       * The decompressed records.
       */
      private WeakReference<Records> recordsRef;

      public Chunk(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         messageStartTime = checkPositiveLong(dataInput.getLong(), "messageStartTime");
         messageEndTime = checkPositiveLong(dataInput.getLong(), "messageEndTime");
         recordsUncompressedLength = checkPositiveLong(dataInput.getLong(), "uncompressedSize");
         uncompressedCrc32 = dataInput.getUnsignedInt();
         compression = dataInput.getString();
         recordsCompressedLength = checkPositiveLong(dataInput.getLong(), "recordsLength");
         recordsOffset = dataInput.position();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return 3 * Long.BYTES + 2 * Integer.BYTES + compression.length() + Long.BYTES + (int) recordsCompressedLength;
      }

      public long messageStartTime()
      {
         return messageStartTime;
      }

      public long messageEndTime()
      {
         return messageEndTime;
      }

      public long uncompressedSize()
      {
         return recordsUncompressedLength;
      }

      /**
       * CRC-32 checksum of uncompressed `records` field. A value of zero indicates that CRC validation
       * should not be performed.
       */
      public long uncompressedCrc32()
      {
         return uncompressedCrc32;
      }

      public String compression()
      {
         return compression;
      }

      public long recordsLength()
      {
         return recordsCompressedLength;
      }

      public Records records()
      {
         Records records = recordsRef == null ? null : recordsRef.get();

         if (records != null)
            return records;

         if (compression.equalsIgnoreCase(""))
         {
            records = new Records(dataInput, recordsOffset, (int) recordsCompressedLength);
         }
         else
         {
            ByteBuffer decompressedBuffer = dataInput.getDecompressedByteBuffer(recordsOffset,
                                                                                (int) recordsCompressedLength,
                                                                                (int) recordsUncompressedLength,
                                                                                Compression.fromString(compression),
                                                                                false);
            records = new Records(MCAPDataInput.wrap(decompressedBuffer), 0, (int) recordsUncompressedLength);
         }

         recordsRef = new WeakReference<>(records);
         return records;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-messageStartTime = " + messageStartTime;
         out += "\n\t-messageEndTime = " + messageEndTime;
         out += "\n\t-compression = " + compression;
         out += "\n\t-recordsCompressedLength = " + recordsCompressedLength;
         out += "\n\t-recordsUncompressedLength = " + recordsUncompressedLength;
         out += "\n\t-uncompressedCrc32 = " + uncompressedCrc32;
         return out;
      }
   }

   public static class DataEnd implements MCAPElement
   {
      private final long dataSectionCrc32;

      public DataEnd(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         dataInput.position(elementPosition);
         dataSectionCrc32 = dataInput.getUnsignedInt();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return Integer.BYTES;
      }

      /**
       * CRC-32 of all bytes in the data section. A value of 0 indicates the CRC-32 is not available.
       */
      public long dataSectionCrc32()
      {
         return dataSectionCrc32;
      }

      @Override
      public String toString()
      {
         return getClass().getSimpleName() + ":\n\t-dataSectionCrc32 = " + dataSectionCrc32;
      }
   }

   public static class Channel implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long elementLength;
      private final int id;
      private final int schemaId;
      private final String topic;
      private final String messageEncoding;
      private WeakReference<List<TupleStrStr>> metadataRef;
      private final long metadataOffset;
      private final long metadataLength;

      public Channel(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;
         this.elementLength = elementLength;

         dataInput.position(elementPosition);
         id = dataInput.getUnsignedShort();
         schemaId = dataInput.getUnsignedShort();
         topic = dataInput.getString();
         messageEncoding = dataInput.getString();
         metadataLength = dataInput.getUnsignedInt();
         metadataOffset = dataInput.position();
      }

      @Override
      public long getElementLength()
      {
         return elementLength;
      }

      public int id()
      {
         return id;
      }

      public int schemaId()
      {
         return schemaId;
      }

      public String topic()
      {
         return topic;
      }

      public String messageEncoding()
      {
         return messageEncoding;
      }

      public List<TupleStrStr> metadata()
      {
         List<TupleStrStr> metadata = metadataRef == null ? null : metadataRef.get();

         if (metadata == null)
         {
            metadata = parseList(dataInput, TupleStrStr::new, metadataOffset, metadataLength);
            metadataRef = new WeakReference<>(metadata);
         }

         return metadata;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-id = " + id;
         out += "\n\t-schemaId = " + schemaId;
         out += "\n\t-topic = " + topic;
         out += "\n\t-messageEncoding = " + messageEncoding;
         out += "\n\t-metadata = [%s]".formatted(metadata().toString());
         return out;
      }
   }

   public static class MessageIndex implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long elementLength;
      private final int channelId;
      private WeakReference<List<MessageIndexEntry>> messageIndexEntriesRef;
      private final long messageIndexEntriesOffset;
      private final long messageIndexEntriesLength;

      public MessageIndex(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;
         this.elementLength = elementLength;

         dataInput.position(elementPosition);
         channelId = dataInput.getUnsignedShort();
         messageIndexEntriesLength = dataInput.getUnsignedInt();
         messageIndexEntriesOffset = dataInput.position();
      }

      @Override
      public long getElementLength()
      {
         return elementLength;
      }

      public static class MessageIndexEntry implements MCAPElement
      {
         /**
          * Time at which the message was recorded.
          */
         private final long logTime;

         /**
          * Offset is relative to the start of the uncompressed chunk data.
          */
         private final long offset;

         public MessageIndexEntry(MCAPDataInput dataInput, long elementPosition)
         {
            dataInput.position(elementPosition);
            logTime = checkPositiveLong(dataInput.getLong(), "logTime");
            offset = checkPositiveLong(dataInput.getLong(), "offset");
         }

         @Override
         public long getElementLength()
         {
            return 2 * Long.BYTES;
         }

         public long logTime()
         {
            return logTime;
         }

         public long offset()
         {
            return offset;
         }

         @Override
         public String toString()
         {
            return toString(0);
         }

         @Override
         public String toString(int indent)
         {
            String out = getClass().getSimpleName() + ":";
            out += "\n\t-logTime = " + logTime;
            out += "\n\t-offset = " + offset;
            return indent(out, indent);
         }
      }

      public int channelId()
      {
         return channelId;
      }

      public List<MessageIndexEntry> messageIndexEntries()
      {
         List<MessageIndexEntry> messageIndexEntries = messageIndexEntriesRef == null ? null : messageIndexEntriesRef.get();

         if (messageIndexEntries == null)
         {
            messageIndexEntries = parseList(dataInput, MessageIndexEntry::new, messageIndexEntriesOffset, messageIndexEntriesLength);
            messageIndexEntriesRef = new WeakReference<>(messageIndexEntries);
         }

         return messageIndexEntries;
      }

      @Override
      public String toString()
      {
         return toString(0);
      }

      @Override
      public String toString(int indent)
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-channelId = " + channelId;
         List<MessageIndexEntry> messageIndexEntries = messageIndexEntries();
         out += "\n\t-messageIndexEntries = " + (messageIndexEntries == null ?
               "null" :
               "\n" + EuclidCoreIOTools.getCollectionString("\n", messageIndexEntries, e -> e.toString(indent + 1)));
         return indent(out, indent);
      }
   }

   public static class Statistics implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long elementLength;
      private final long messageCount;
      private final int schemaCount;
      private final long channelCount;
      private final long attachmentCount;
      private final long metadataCount;
      private final long chunkCount;
      private final long messageStartTime;
      private final long messageEndTime;
      private WeakReference<List<ChannelMessageCount>> channelMessageCountsRef;
      private final long channelMessageCountsOffset;
      private final long channelMessageCountsLength;

      public Statistics(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;
         this.elementLength = elementLength;

         dataInput.position(elementPosition);
         messageCount = checkPositiveLong(dataInput.getLong(), "messageCount");
         schemaCount = dataInput.getUnsignedShort();
         channelCount = dataInput.getUnsignedInt();
         attachmentCount = dataInput.getUnsignedInt();
         metadataCount = dataInput.getUnsignedInt();
         chunkCount = dataInput.getUnsignedInt();
         messageStartTime = checkPositiveLong(dataInput.getLong(), "messageStartTime");
         messageEndTime = checkPositiveLong(dataInput.getLong(), "messageEndTime");
         channelMessageCountsLength = dataInput.getUnsignedInt();
         channelMessageCountsOffset = dataInput.position();
      }

      @Override
      public long getElementLength()
      {
         return elementLength;
      }

      public static class ChannelMessageCount implements MCAPElement
      {
         private final int channelId;
         private final long messageCount;

         public ChannelMessageCount(MCAPDataInput dataInput, long elementPosition)
         {
            dataInput.position(elementPosition);
            channelId = dataInput.getUnsignedShort();
            messageCount = dataInput.getLong();
         }

         @Override
         public long getElementLength()
         {
            return Short.BYTES + Long.BYTES;
         }

         public int channelId()
         {
            return channelId;
         }

         public long messageCount()
         {
            return messageCount;
         }

         @Override
         public String toString()
         {
            return toString(0);
         }

         @Override
         public String toString(int indent)
         {
            String out = getClass().getSimpleName() + ":";
            out += "\n\t-channelId = " + channelId;
            out += "\n\t-messageCount = " + messageCount;
            return indent(out, indent);
         }
      }

      public long messageCount()
      {
         return messageCount;
      }

      public int schemaCount()
      {
         return schemaCount;
      }

      public long channelCount()
      {
         return channelCount;
      }

      public long attachmentCount()
      {
         return attachmentCount;
      }

      public long metadataCount()
      {
         return metadataCount;
      }

      public long chunkCount()
      {
         return chunkCount;
      }

      public long messageStartTime()
      {
         return messageStartTime;
      }

      public long messageEndTime()
      {
         return messageEndTime;
      }

      public List<ChannelMessageCount> channelMessageCounts()
      {
         List<ChannelMessageCount> channelMessageCounts = channelMessageCountsRef == null ? null : channelMessageCountsRef.get();

         if (channelMessageCounts == null)
         {
            channelMessageCounts = parseList(dataInput, ChannelMessageCount::new, channelMessageCountsOffset, channelMessageCountsLength);
            channelMessageCountsRef = new WeakReference<>(channelMessageCounts);
         }

         return channelMessageCounts;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-messageCount = " + messageCount;
         out += "\n\t-schemaCount = " + schemaCount;
         out += "\n\t-channelCount = " + channelCount;
         out += "\n\t-attachmentCount = " + attachmentCount;
         out += "\n\t-metadataCount = " + metadataCount;
         out += "\n\t-chunkCount = " + chunkCount;
         out += "\n\t-messageStartTime = " + messageStartTime;
         out += "\n\t-messageEndTime = " + messageEndTime;
         out += "\n\t-channelMessageCounts = \n" + EuclidCoreIOTools.getCollectionString("\n", channelMessageCounts(), e -> e.toString(1));
         return out;
      }
   }

   public static class AttachmentIndex implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long attachmentOffset;
      private final long attachmentLength;
      private final long logTime;
      private final long createTime;
      private final long dataSize;
      private final String name;
      private final String mediaType;

      private WeakReference<Record> attachmentRef;

      private AttachmentIndex(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         attachmentOffset = checkPositiveLong(dataInput.getLong(), "attachmentOffset");
         attachmentLength = checkPositiveLong(dataInput.getLong(), "attachmentLength");
         logTime = checkPositiveLong(dataInput.getLong(), "logTime");
         createTime = checkPositiveLong(dataInput.getLong(), "createTime");
         dataSize = checkPositiveLong(dataInput.getLong(), "dataSize");
         name = dataInput.getString();
         mediaType = dataInput.getString();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return 5 * Long.BYTES + 2 * Integer.BYTES + name.length() + mediaType.length();
      }

      public Record attachment()
      {
         Record attachment = attachmentRef == null ? null : attachmentRef.get();

         if (attachment == null)
         {
            attachment = new Record(dataInput, attachmentOffset);
            attachmentRef = new WeakReference<>(attachment);
         }

         return attachment;
      }

      public long attachmentOffset()
      {
         return attachmentOffset;
      }

      public long attachmentLength()
      {
         return attachmentLength;
      }

      public long logTime()
      {
         return logTime;
      }

      public long createTime()
      {
         return createTime;
      }

      public long dataSize()
      {
         return dataSize;
      }

      public String name()
      {
         return name;
      }

      public String mediaType()
      {
         return mediaType;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-attachmentOffset = " + attachmentOffset;
         out += "\n\t-attachmentLength = " + attachmentLength;
         out += "\n\t-logTime = " + logTime;
         out += "\n\t-createTime = " + createTime;
         out += "\n\t-dataSize = " + dataSize;
         out += "\n\t-name = " + name;
         out += "\n\t-mediaType = " + mediaType;
         return out;
      }
   }

   public static class Schema implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final int id;
      private final String name;
      private final String encoding;
      private final long dataLength;
      private final long dataOffset;
      private WeakReference<ByteBuffer> dataRef;

      public Schema(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         id = dataInput.getUnsignedShort();
         name = dataInput.getString();
         encoding = dataInput.getString();
         dataLength = dataInput.getUnsignedInt();
         dataOffset = dataInput.position();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return Short.BYTES + 3 * Integer.BYTES + name.length() + encoding.length() + (int) dataLength;
      }

      public int id()
      {
         return id;
      }

      public String name()
      {
         return name;
      }

      public String encoding()
      {
         return encoding;
      }

      public ByteBuffer data()
      {
         ByteBuffer data = this.dataRef == null ? null : this.dataRef.get();

         if (data == null)
         {
            data = dataInput.getByteBuffer(dataOffset, (int) dataLength, false);
            dataRef = new WeakReference<>(data);
         }
         return data;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-id = " + id;
         out += "\n\t-name = " + name;
         out += "\n\t-encoding = " + encoding;
         out += "\n\t-dataLength = " + dataLength;
         out += "\n\t-data = " + Arrays.toString(data().array());
         return out;
      }
   }

   public static class SummaryOffset implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final Opcode groupOpcode;
      private final long offsetGroup;
      private final long lengthGroup;

      private WeakReference<Records> groupRef;

      public SummaryOffset(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         groupOpcode = Opcode.byId(dataInput.getUnsignedByte());
         offsetGroup = checkPositiveLong(dataInput.getLong(), "offsetGroup");
         lengthGroup = checkPositiveLong(dataInput.getLong(), "lengthGroup");
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return Byte.BYTES + 2 * Long.BYTES;
      }

      public Records group()
      {
         Records group = groupRef == null ? null : groupRef.get();

         if (group == null)
         {
            group = new Records(dataInput, offsetGroup, (int) lengthGroup);
            groupRef = new WeakReference<>(group);
         }
         return group;
      }

      public Opcode groupOpcode()
      {
         return groupOpcode;
      }

      public long offsetGroup()
      {
         return offsetGroup;
      }

      public long lengthGroup()
      {
         return lengthGroup;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-groupOpcode = " + groupOpcode;
         out += "\n\t-offsetGroup = " + offsetGroup;
         out += "\n\t-lengthGroup = " + lengthGroup;
         return out;
      }
   }

   public static class Attachment implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long logTime;
      private final long createTime;
      private final String name;
      private final String mediaType;
      private final long lengthData;
      private final long offsetData;
      private WeakReference<ByteBuffer> dataRef;
      private final long crc32;
      private final long crc32InputStart;
      private final int crc32InputLength;
      private WeakReference<ByteBuffer> crc32InputRef;

      private Attachment(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         crc32InputStart = elementPosition;
         logTime = checkPositiveLong(dataInput.getLong(), "logTime");
         createTime = checkPositiveLong(dataInput.getLong(), "createTime");
         name = dataInput.getString();
         mediaType = dataInput.getString();
         lengthData = checkPositiveLong(dataInput.getLong(), "lengthData");
         offsetData = dataInput.position();
         dataInput.skip(lengthData);
         crc32InputLength = (int) (dataInput.position() - elementPosition);
         crc32 = dataInput.getUnsignedInt();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return 3 * Long.BYTES + 3 * Integer.BYTES + name.length() + mediaType.length() + (int) lengthData;
      }

      public ByteBuffer crc32Input()
      {
         ByteBuffer crc32Input = this.crc32InputRef == null ? null : this.crc32InputRef.get();

         if (crc32Input == null)
         {
            crc32Input = dataInput.getByteBuffer(crc32InputStart, crc32InputLength, false);
            crc32InputRef = new WeakReference<>(crc32Input);
         }

         return crc32Input;
      }

      public long logTime()
      {
         return logTime;
      }

      public long createTime()
      {
         return createTime;
      }

      public String name()
      {
         return name;
      }

      public String mediaType()
      {
         return mediaType;
      }

      public long lenData()
      {
         return lengthData;
      }

      public ByteBuffer data()
      {
         ByteBuffer data = this.dataRef == null ? null : this.dataRef.get();

         if (data == null)
         {
            data = dataInput.getByteBuffer(offsetData, (int) lengthData, false);
            dataRef = new WeakReference<>(data);
         }
         return data;
      }

      public void unloadData()
      {
         dataRef = null;
      }

      /**
       * CRC-32 checksum of preceding fields in the record. A value of zero indicates that CRC validation
       * should not be performed.
       */
      public long crc32()
      {
         return crc32;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-logTime = " + logTime;
         out += "\n\t-createTime = " + createTime;
         out += "\n\t-name = " + name;
         out += "\n\t-mediaType = " + mediaType;
         out += "\n\t-lengthData = " + lengthData;
         //         out += "\n\t-data = " + data;
         out += "\n\t-crc32 = " + crc32;
         return out;
      }
   }

   public static class Metadata implements MCAPElement
   {
      private final String name;
      private final List<TupleStrStr> metadata;
      private final int metadataLength;

      private Metadata(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         dataInput.position(elementPosition);
         name = dataInput.getString();
         long start = dataInput.position();
         metadata = parseList(dataInput, TupleStrStr::new); // TODO Looks into postponing the loading of the metadata.
         metadataLength = (int) (dataInput.position() - start);
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return Integer.BYTES + name.length() + metadataLength;
      }

      public String name()
      {
         return name;
      }

      public List<TupleStrStr> metadata()
      {
         return metadata;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-name = " + name;
         out += "\n\t-metadata = " + EuclidCoreIOTools.getCollectionString(", ", metadata, e -> e.key());
         return out;
      }
   }

   public static class Header implements MCAPElement
   {
      private final String profile;
      private final String library;

      public Header(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         dataInput.position(elementPosition);
         profile = dataInput.getString();
         library = dataInput.getString();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return 2 * Integer.BYTES + profile.length() + library.length();
      }

      public String profile()
      {
         return profile;
      }

      public String library()
      {
         return library;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-profile = " + profile;
         out += "\n\t-library = " + library;
         return out;
      }
   }

   public static class Message implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private int channelId;
      private long sequence;
      private long logTime;
      private long publishTime;
      private long dataOffset;
      private int dataLength;
      private WeakReference<ByteBuffer> messageBufferRef;
      private WeakReference<byte[]> messageDataRef;

      public static Message createSpoofMessageForTesting(int channelId, byte[] data)
      {
         return new Message()
         {
            @Override
            public int channelId()
            {
               return channelId;
            }

            @Override
            public long dataOffset()
            {
               return 0;
            }

            @Override
            public int dataLength()
            {
               return data.length;
            }

            @Override
            public ByteBuffer messageBuffer()
            {
               return ByteBuffer.wrap(data);
            }

            @Override
            public byte[] messageData()
            {
               return data;
            }
         };
      }

      private Message()
      {
         dataInput = null;
      }

      private Message(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         channelId = dataInput.getUnsignedShort();
         sequence = dataInput.getUnsignedInt();
         logTime = checkPositiveLong(dataInput.getLong(), "logTime");
         publishTime = checkPositiveLong(dataInput.getLong(), "publishTime");
         dataOffset = dataInput.position();
         dataLength = (int) (elementLength - (Short.BYTES + Integer.BYTES + 2 * Long.BYTES));
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return dataLength + Short.BYTES + Integer.BYTES + 2 * Long.BYTES;
      }

      public int channelId()
      {
         return channelId;
      }

      public long sequence()
      {
         return sequence;
      }

      public long logTime()
      {
         return logTime;
      }

      public long publishTime()
      {
         return publishTime;
      }

      /**
       * Returns the offset of the data portion of this message in the buffer returned by
       * {@link #messageBuffer()}.
       *
       * @return the offset of the data portion of this message.
       */
      public long dataOffset()
      {
         return dataOffset;
      }

      /**
       * Returns the length of the data portion of this message.
       *
       * @return the length of the data portion of this message.
       */
      public int dataLength()
      {
         return dataLength;
      }

      /**
       * Returns the buffer containing this message, the data AND the header. Use {@link #dataOffset()}
       * and {@link #dataLength()} to get the data portion.
       *
       * @return the buffer containing this message.
       */
      public ByteBuffer messageBuffer()
      {
         ByteBuffer messageBuffer = messageBufferRef == null ? null : messageBufferRef.get();
         if (messageBuffer == null)
         {
            messageBuffer = dataInput.getByteBuffer(dataOffset, dataLength, false);
            messageBufferRef = new WeakReference<>(messageBuffer);
         }
         return messageBuffer;
      }

      public byte[] messageData()
      {
         byte[] messageData = messageDataRef == null ? null : messageDataRef.get();

         if (messageData == null)
         {
            messageData = dataInput.getBytes(dataOffset, dataLength);
            messageDataRef = new WeakReference<>(messageData);
         }
         return messageData;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-channelId = " + channelId;
         out += "\n\t-sequence = " + sequence;
         out += "\n\t-logTime = " + logTime;
         out += "\n\t-publishTime = " + publishTime;
         //         out += "\n\t-data = " + data;
         return out;
      }
   }

   public static class TupleStrStr implements MCAPElement
   {
      private final String key;
      private final String value;

      public TupleStrStr(MCAPDataInput dataInput, long elementPosition)
      {
         dataInput.position(elementPosition);
         key = dataInput.getString();
         value = dataInput.getString();
      }

      @Override
      public long getElementLength()
      {
         return key.length() + value.length() + 2 * Integer.BYTES;
      }

      public String key()
      {
         return key;
      }

      public String value()
      {
         return value;
      }

      @Override
      public String toString()
      {
         return (key + ": " + value).replace("\n", "");
      }
   }

   public static class MetadataIndex implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long metadataOffset;
      private final long metadataLength;
      private final String name;
      private WeakReference<Record> metadataRef;

      private MetadataIndex(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         metadataOffset = checkPositiveLong(dataInput.getLong(), "metadataOffset");
         metadataLength = checkPositiveLong(dataInput.getLong(), "metadataLength");
         name = dataInput.getString();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return 2 * Long.BYTES + Integer.BYTES + name.length();
      }

      public Record metadata()
      {
         Record metadata = metadataRef == null ? null : metadataRef.get();

         if (metadata == null)
         {
            metadata = new Record(dataInput, metadataOffset);
            metadataRef = new WeakReference<>(metadata);
         }
         return metadata;
      }

      public long metadataOffset()
      {
         return metadataOffset;
      }

      public long metadataLength()
      {
         return metadataLength;
      }

      public String name()
      {
         return name;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-metadataOffset = " + metadataOffset;
         out += "\n\t-metadataLength = " + metadataLength;
         out += "\n\t-name = " + name;
         return out;
      }
   }

   public static class Magic implements MCAPElement
   {
      public static final int MAGIC_SIZE = 8;
      public static final byte[] MAGIC_BYTES = {-119, 77, 67, 65, 80, 48, 13, 10};

      private final byte[] magic;

      public Magic(MCAPDataInput dataInput, long elementPosition)
      {
         dataInput.position(elementPosition);
         magic = dataInput.getBytes(MAGIC_SIZE);
         if (!(Arrays.equals(magic, MAGIC_BYTES)))
            throw new IllegalArgumentException("Invalid magic bytes: " + Arrays.toString(magic) + ". Expected: " + Arrays.toString(MAGIC_BYTES));
      }

      @Override
      public long getElementLength()
      {
         return MAGIC_SIZE;
      }

      public byte[] magic()
      {
         return magic;
      }

      @Override
      public String toString()
      {
         return toString(0);
      }

      @Override
      public String toString(int indent)
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-magic = " + Arrays.toString(magic);
         return indent(out, indent);
      }
   }

   public static class Records extends ArrayList<Record>
   {
      public Records(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         parseList(dataInput, Record::new, elementPosition, elementLength, this);
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
   }

   public static class Footer implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long ofsSummarySection;
      private final long ofsSummaryOffsetSection;
      private final long summaryCrc32;
      private Integer ofsSummaryCrc32Input;
      private Records summaryOffsetSection;
      private Records summarySection;

      public Footer(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         ofsSummarySection = checkPositiveLong(dataInput.getLong(), "ofsSummarySection");
         ofsSummaryOffsetSection = checkPositiveLong(dataInput.getLong(), "ofsSummaryOffsetSection");
         summaryCrc32 = dataInput.getUnsignedInt();
         checkLength(elementLength, getElementLength());
      }

      @Override
      public long getElementLength()
      {
         return 2 * Long.BYTES + Integer.BYTES;
      }

      public Records summarySection()
      {
         if (summarySection == null && ofsSummarySection != 0)
         {
            long length = ((ofsSummaryOffsetSection != 0 ? ofsSummaryOffsetSection : computeOffsetFooter(dataInput)) - ofsSummarySection);
            summarySection = new Records(dataInput, ofsSummarySection, (int) length);
         }
         return summarySection;
      }

      public Records summaryOffsetSection()
      {
         if (summaryOffsetSection == null && ofsSummaryOffsetSection != 0)
         {
            summaryOffsetSection = new Records(dataInput, ofsSummaryOffsetSection, (int) (computeOffsetFooter(dataInput) - ofsSummaryOffsetSection));
         }
         return summaryOffsetSection;
      }

      public Integer ofsSummaryCrc32Input()
      {
         if (ofsSummaryCrc32Input == null)
         {
            ofsSummaryCrc32Input = (int) ((ofsSummarySection() != 0 ? ofsSummarySection() : computeOffsetFooter(dataInput)));
         }
         return ofsSummaryCrc32Input;
      }

      private byte[] summaryCrc32Input;

      public byte[] summaryCrc32Input()
      {
         if (summaryCrc32Input == null)
         {
            long length = dataInput.size() - ofsSummaryCrc32Input() - 8 - 4;
            summaryCrc32Input = dataInput.getBytes(ofsSummaryCrc32Input(), (int) length);
         }
         return summaryCrc32Input;
      }

      public long ofsSummarySection()
      {
         return ofsSummarySection;
      }

      public long ofsSummaryOffsetSection()
      {
         return ofsSummaryOffsetSection;
      }

      /**
       * A CRC-32 of all bytes from the start of the Summary section up through and including the end of
       * the previous field (summary_offset_start) in the footer record. A value of 0 indicates the CRC-32
       * is not available.
       */
      public long summaryCrc32()
      {
         return summaryCrc32;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-ofsSummarySection = " + ofsSummarySection;
         out += "\n\t-ofsSummaryOffsetSection = " + ofsSummaryOffsetSection;
         out += "\n\t-summaryCrc32 = " + summaryCrc32;
         return out;
      }
   }

   public static class Record implements MCAPElement
   {
      public static final int RECORD_HEADER_LENGTH = 9;

      private final MCAPDataInput dataInput;

      private final Opcode op;
      private final long bodyLength;
      private final long bodyOffset;
      private WeakReference<Object> bodyRef;

      public Record(MCAPDataInput dataInput)
      {
         this(dataInput, dataInput.position());
      }

      public Record(MCAPDataInput dataInput, long elementPosition)
      {
         this.dataInput = dataInput;

         dataInput.position(elementPosition);
         op = Opcode.byId(dataInput.getUnsignedByte());
         bodyLength = checkPositiveLong(dataInput.getLong(), "bodyLength");
         bodyOffset = dataInput.position();
         checkLength(getElementLength(), (int) (bodyLength + RECORD_HEADER_LENGTH));
      }

      public Opcode op()
      {
         return op;
      }

      public long bodyLength()
      {
         return bodyLength;
      }

      public Object body()
      {
         Object body = bodyRef == null ? null : bodyRef.get();

         if (body == null)
         {
            if (op == null)
            {
               body = dataInput.getBytes(bodyOffset, (int) bodyLength);
            }
            else
            {
               body = switch (op)
               {
                  case MESSAGE -> new Message(dataInput, bodyOffset, bodyLength);
                  case METADATA_INDEX -> new MetadataIndex(dataInput, bodyOffset, bodyLength);
                  case CHUNK -> new Chunk(dataInput, bodyOffset, bodyLength);
                  case SCHEMA -> new Schema(dataInput, bodyOffset, bodyLength);
                  case CHUNK_INDEX -> new ChunkIndex(dataInput, bodyOffset, bodyLength);
                  case DATA_END -> new DataEnd(dataInput, bodyOffset, bodyLength);
                  case ATTACHMENT_INDEX -> new AttachmentIndex(dataInput, bodyOffset, bodyLength);
                  case STATISTICS -> new Statistics(dataInput, bodyOffset, bodyLength);
                  case MESSAGE_INDEX -> new MessageIndex(dataInput, bodyOffset, bodyLength);
                  case CHANNEL -> new Channel(dataInput, bodyOffset, bodyLength);
                  case METADATA -> new Metadata(dataInput, bodyOffset, bodyLength);
                  case ATTACHMENT -> new Attachment(dataInput, bodyOffset, bodyLength);
                  case HEADER -> new Header(dataInput, bodyOffset, bodyLength);
                  case FOOTER -> new Footer(dataInput, bodyOffset, bodyLength);
                  case SUMMARY_OFFSET -> new SummaryOffset(dataInput, bodyOffset, bodyLength);
               };
            }

            bodyRef = new WeakReference<>(body);
         }
         return body;
      }

      @Override
      public long getElementLength()
      {
         return RECORD_HEADER_LENGTH + bodyLength;
      }

      @Override
      public String toString()
      {
         return toString(0);
      }

      @Override
      public String toString(int indent)
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-op = " + op;
         out += "\n\t-bodyLength = " + bodyLength;
         out += "\n\t-bodyOffset = " + bodyOffset;
         Object body = body();
         out += "\n\t-body = " + (body == null ? "null" : "\n" + ((MCAPElement) body).toString(indent + 2));
         return indent(out, indent);
      }
   }

   public static class ChunkIndex implements MCAPElement
   {
      private final MCAPDataInput dataInput;
      private final long elementLength;
      /**
       * Earliest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private final long messageStartTime;
      /**
       * Latest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private final long messageEndTime;
      /**
       * Offset to the chunk record from the start of the file.
       */
      private final long chunkOffset;
      /**
       * Byte length of the chunk record, including opcode and length prefix.
       */
      private final long chunkLength;
      private final long messageIndexOffsetsOffset;
      /**
       * Total length in bytes of the message index records after the chunk.
       */
      private final long messageIndexOffsetsLength;
      /**
       * Mapping from channel ID to the offset of the message index record for that channel after the
       * chunk, from the start of the file. An empty map indicates no message indexing is available.
       */
      private WeakReference<MessageIndexOffsets> messageIndexOffsetsRef;
      /**
       * Total length in bytes of the message index records after the chunk.
       */
      private final long messageIndexLength;
      /**
       * The compression used within the chunk. Refer to well-known compression formats. This field should
       * match the the value in the corresponding Chunk record.
       */
      private final String compression;
      /**
       * The size of the chunk records field.
       */
      private final long compressedSize;
      /**
       * The uncompressed size of the chunk records field. This field should match the value in the
       * corresponding Chunk record.
       */
      private final long uncompressedSize;

      private ChunkIndex(MCAPDataInput dataInput, long elementPosition, long elementLength)
      {
         this.dataInput = dataInput;
         this.elementLength = elementLength;

         dataInput.position(elementPosition);
         messageStartTime = checkPositiveLong(dataInput.getLong(), "messageStartTime");
         messageEndTime = checkPositiveLong(dataInput.getLong(), "messageEndTime");
         chunkOffset = checkPositiveLong(dataInput.getLong(), "chunkOffset");
         chunkLength = checkPositiveLong(dataInput.getLong(), "chunkLength");
         messageIndexOffsetsLength = dataInput.getUnsignedInt();
         messageIndexOffsetsOffset = dataInput.position();
         dataInput.skip(messageIndexOffsetsLength);
         messageIndexLength = checkPositiveLong(dataInput.getLong(), "messageIndexLength");
         compression = dataInput.getString();
         compressedSize = checkPositiveLong(dataInput.getLong(), "compressedSize");
         uncompressedSize = checkPositiveLong(dataInput.getLong(), "uncompressedSize");
      }

      @Override
      public long getElementLength()
      {
         return elementLength;
      }

      public static class MessageIndexOffset implements MCAPElement
      {
         /**
          * Channel ID.
          */
         private final int channelId;
         /**
          * Offset of the message index record for that channel after the chunk, from the start of the file.
          */
         private final long offset;

         public MessageIndexOffset(MCAPDataInput dataInput, long elementPosition)
         {
            dataInput.position(elementPosition);
            channelId = dataInput.getUnsignedShort();
            offset = checkPositiveLong(dataInput.getLong(), "offset");
         }

         @Override
         public long getElementLength()
         {
            return Short.BYTES + Long.BYTES;
         }

         public int channelId()
         {
            return channelId;
         }

         public long offset()
         {
            return offset;
         }

         @Override
         public String toString()
         {
            return toString(0);
         }

         @Override
         public String toString(int indent)
         {
            String out = getClass().getSimpleName() + ":";
            out += "\n\t-channelId = " + channelId;
            out += "\n\t-offset = " + offset;
            return indent(out, indent);
         }
      }

      public static class MessageIndexOffsets implements MCAPElement
      {
         private final List<MessageIndexOffset> entries;
         private final long elementLength;

         public MessageIndexOffsets(MCAPDataInput dataInput, long elementPosition, long elementLength)
         {
            this.elementLength = elementLength;

            entries = new ArrayList<>();

            long currentPos = elementPosition;
            long remaining = elementLength;

            while (remaining > 0)
            {
               MessageIndexOffset entry = new MessageIndexOffset(dataInput, currentPos);
               entries.add(entry);
               currentPos += entry.getElementLength();
               remaining -= entry.getElementLength();
            }

            if (remaining != 0)
               throw new IllegalArgumentException(
                     "Invalid element length. Expected: " + elementLength + ", remaining: " + remaining + ", entries: " + entries.size());
         }

         @Override
         public long getElementLength()
         {
            return elementLength;
         }

         public List<MessageIndexOffset> entries()
         {
            return entries;
         }

         @Override
         public String toString()
         {
            return toString(0);
         }

         @Override
         public String toString(int indent)
         {
            String out = getClass().getSimpleName() + ":";
            out += "\n\t-entries = " + (entries == null ? "null" : "\n" + EuclidCoreIOTools.getCollectionString("\n", entries, e -> e.toString(indent + 1)));
            return indent(out, indent);
         }
      }

      private WeakReference<Record> chunkRef;

      public Record chunk()
      {
         Record chunk = chunkRef == null ? null : chunkRef.get();

         if (chunk == null)
         {
            chunk = new Record(dataInput, chunkOffset);
            chunkRef = new WeakReference<>(chunk);
         }
         return chunkRef.get();
      }

      public long messageStartTime()
      {
         return messageStartTime;
      }

      public long messageEndTime()
      {
         return messageEndTime;
      }

      public long chunkOffset()
      {
         return chunkOffset;
      }

      public long chunkLength()
      {
         return chunkLength;
      }

      public long messageIndexOffsetsLength()
      {
         return messageIndexOffsetsLength;
      }

      public MessageIndexOffsets messageIndexOffsets()
      {
         MessageIndexOffsets messageIndexOffsets = messageIndexOffsetsRef == null ? null : messageIndexOffsetsRef.get();

         if (messageIndexOffsets == null)
         {
            messageIndexOffsets = new MessageIndexOffsets(dataInput, messageIndexOffsetsOffset, messageIndexOffsetsLength);
            messageIndexOffsetsRef = new WeakReference<>(messageIndexOffsets);
         }

         return messageIndexOffsets;
      }

      public long messageIndexLength()
      {
         return messageIndexLength;
      }

      public String compression()
      {
         return compression;
      }

      public long compressedSize()
      {
         return compressedSize;
      }

      public long uncompressedSize()
      {
         return uncompressedSize;
      }

      @Override
      public String toString()
      {
         return toString(0);
      }

      @Override
      public String toString(int indent)
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-messageStartTime = " + messageStartTime;
         out += "\n\t-messageEndTime = " + messageEndTime;
         out += "\n\t-chunkOffset = " + chunkOffset;
         out += "\n\t-chunkLength = " + chunkLength;
         out += "\n\t-messageIndexOffsetsLength = " + messageIndexOffsetsLength;
         //         out += "\n\t-messageIndexOffsets = " + (messageIndexOffsets == null ? "null" : "\n" + messageIndexOffsets.toString(indent + 1));
         out += "\n\t-messageIndexLength = " + messageIndexLength;
         out += "\n\t-compression = " + compression;
         out += "\n\t-compressedSize = " + compressedSize;
         out += "\n\t-uncompressedSize = " + uncompressedSize;
         return indent(out, indent);
      }
   }

   public interface MCAPElement
   {
      long getElementLength();

      default String toString(int indent)
      {
         return indent(toString(), indent);
      }
   }

   public static long computeOffsetFooter(MCAPDataInput dataInput)
   {
      return (((((dataInput.size() - 1L) - 8L) - 20L) - 8L));
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput, MCAPDataReader<T> elementParser)
   {
      return parseList(dataInput, elementParser, dataInput.getUnsignedInt());
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput, MCAPDataReader<T> elementParser, long length)
   {
      return parseList(dataInput, elementParser, dataInput.position(), length);
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput, MCAPDataReader<T> elementParser, long offset, long length)
   {
      return parseList(dataInput, elementParser, offset, length, null);
   }

   public static <T extends MCAPElement> List<T> parseList(MCAPDataInput dataInput,
                                                           MCAPDataReader<T> elementParser,
                                                           long offset,
                                                           long length,
                                                           List<T> listToPack)
   {
      long position = offset;
      long limit = position + length;
      if (listToPack == null)
         listToPack = new ArrayList<>();

      while (position < limit)
      {
         T parsed = elementParser.parse(dataInput, position);
         listToPack.add(parsed);
         position += parsed.getElementLength();
      }

      return listToPack;
   }

   public interface MCAPDataReader<T extends MCAPElement>
   {
      T parse(MCAPDataInput dataInput, long position);
   }

   private static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = "\t".repeat(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
   }

   private static int checkPositiveInt(int value, String name)
   {
      if (value < 0)
         throw new IllegalArgumentException(name + " must be positive. Value: " + value);
      return value;
   }

   private static long checkPositiveLong(long value, String name)
   {
      if (value < 0)
         throw new IllegalArgumentException(name + " must be positive. Value: " + value);
      return value;
   }

   private static void checkLength(long expectedLength, long actualLength)
   {
      if (actualLength != expectedLength)
         throw new IllegalArgumentException("Unexpected length: expected= " + expectedLength + ", actual= " + actualLength);
   }
}