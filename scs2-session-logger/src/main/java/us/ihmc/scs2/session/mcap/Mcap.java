package us.ihmc.scs2.session.mcap;

import java.io.IOException;
import java.io.Serial;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

/**
 * MCAP is a modular container format and logging library for pub/sub messages with arbitrary
 * message serialization. It is primarily intended for use in robotics applications, and works well
 * under various workloads, resource constraints, and durability requirements. Time values
 * (`log_time`, `publish_time`, `create_time`) are represented in nanoseconds since a
 * user-understood epoch (i.e. Unix epoch, robot boot time, etc.)
 *
 * @see <a href="https://github.com/foxglove/mcap/tree/c1cc51d/docs/specification#readme">Source</a>
 */
public class Mcap
{
   /**
    * Stream object that this KaitaiStruct-based structure was parsed from.
    */
   protected FileChannel fileChannel;

   public enum Opcode
   {
      HEADER(1), FOOTER(2), SCHEMA(3), CHANNEL(4), MESSAGE(5), CHUNK(6), MESSAGE_INDEX(7), CHUNK_INDEX(8), ATTACHMENT(9), ATTACHMENT_INDEX(10), STATISTICS(11), METADATA(
         12), METADATA_INDEX(13), SUMMARY_OFFSET(14), DATA_END(15);

      private final long id;

      Opcode(long id)
      {
         this.id = id;
      }

      public long id()
      {
         return id;
      }

      private static final Map<Long, Opcode> byId = new HashMap<>(15);

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

   public Mcap(FileChannel fileChannel) throws IOException
   {
      this.fileChannel = fileChannel;
      _read();
   }

   public FileChannel getFileChannel()
   {
      return fileChannel;
   }

   private void _read() throws IOException
   {
      long currentPos = 0;
      headerMagic = new Magic(fileChannel, currentPos);
      currentPos += headerMagic.getItemTotalLength();
      records = new ArrayList<>();
      Record lastRecord = null;

      do
      {
         lastRecord = new Record(fileChannel, currentPos);
         currentPos += lastRecord.getItemTotalLength();
         records.add(lastRecord);
      }
      while (!(lastRecord.op() == Opcode.FOOTER));

      footerMagic = new Magic(fileChannel, currentPos);
   }

   private static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = "\t".repeat(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
   }

   public static class PrefixedStr extends KaitaiStruct
   {
      private long lenStr;
      private String str;

      public PrefixedStr(ByteBuffer buffer) throws IOException
      {
         super(buffer);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         lenStr = Integer.toUnsignedLong(buffer.getInt());
         str = new String(buffer.array(), buffer.position(), (int) lenStr, StandardCharsets.UTF_8);
         buffer.position(buffer.position() + (int) lenStr); // Make sure we advance the buffer position to skip the string.
         setComputedLength((int) lenStr + Integer.BYTES);
      }

      public long lenStr()
      {
         return lenStr;
      }

      public String str()
      {
         return str;
      }

      @Override
      public String toString()
      {
         return str;
      }
   }

   public static class Chunk extends KaitaiStruct
   {
      private LZ4FrameDecoder lz4FrameDecoder;

      /**
       * Earliest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private long messageStartTime;
      /**
       * Latest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private long messageEndTime;
      /**
       * Uncompressed size of the records field.
       */
      private long uncompressedSize;
      /**
       * CRC32 checksum of uncompressed records field. A value of zero indicates that CRC validation should not be performed.
       */
      private long uncompressedCrc32;
      /**
       * compression algorithm. i.e. zstd, lz4, "". An empty string indicates no compression. Refer to well-known compression formats.
       */
      private PrefixedStr compression;
      /**
       * Offset position of the records in either in the {@code  ByteBuffer} or {@code FileChannel}, depending how this chunk was created.
       */
      private long offsetRecords;
      /**
       * Length of the records in bytes.
       */
      private long lengthRecords;
      /**
       * The decompressed records.
       */
      private Records records;

      public Chunk(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Chunk(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         messageStartTime = buffer.getLong();
         messageEndTime = buffer.getLong();
         uncompressedSize = buffer.getLong();
         uncompressedCrc32 = Integer.toUnsignedLong(buffer.getInt());
         compression = new PrefixedStr(buffer);
         lengthRecords = buffer.getLong();
         offsetRecords = buffer.position();
         buffer.position((int) (offsetRecords + lengthRecords)); // Skip the records.

         setComputedLength(3 * Long.BYTES + Integer.BYTES + compression.getItemTotalLength() + Long.BYTES + (int) lengthRecords);
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
         return uncompressedSize;
      }

      /**
       * CRC-32 checksum of uncompressed `records` field. A value of zero indicates that CRC validation
       * should not be performed.
       */
      public long uncompressedCrc32()
      {
         return uncompressedCrc32;
      }

      public PrefixedStr compression()
      {
         return compression;
      }

      public long lenRecords()
      {
         return lengthRecords;
      }

      public Records records() throws IOException
      {
         if (records == null)
         {
            if (compression.str().equalsIgnoreCase(""))
            {
               records = new Records(buffer, offsetRecords, (int) lengthRecords);
            }
            else if (compression.str().equalsIgnoreCase("lz4"))
            {
               if (lz4FrameDecoder == null)
                  lz4FrameDecoder = new LZ4FrameDecoder();
               ByteBuffer decompressedData = ByteBuffer.allocate((int) uncompressedSize);
               lz4FrameDecoder.decode(buffer, (int) offsetRecords, (int) lengthRecords, decompressedData, 0);
               records = new Records(decompressedData);
            }
            else
            {
               throw new UnsupportedOperationException("Unsupported compression algorithm: " + compression.str());
            }
         }
         return records;
      }

      public void unloadRecords()
      {
         records = null;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-messageStartTime = " + messageStartTime;
         out += "\n\t-messageEndTime = " + messageEndTime;
         out += "\n\t-compression = " + compression;
         out += "\n\t-compressedSize = " + lengthRecords;
         out += "\n\t-uncompressedSize = " + uncompressedSize;
         out += "\n\t-uncompressedCrc32 = " + uncompressedCrc32;
         return out;
      }
   }

   public static class DataEnd extends KaitaiStruct
   {
      private long dataSectionCrc32;

      public DataEnd(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public DataEnd(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         dataSectionCrc32 = Integer.toUnsignedLong(buffer.getInt());
         setComputedLength(Integer.BYTES);
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

   public static class Channel extends KaitaiStruct
   {
      private int id;
      private int schemaId;
      private PrefixedStr topic;
      private PrefixedStr messageEncoding;
      private MapStrStr metadata;

      public Channel(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Channel(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         id = Short.toUnsignedInt(buffer.getShort());
         schemaId = Short.toUnsignedInt(buffer.getShort());
         topic = new PrefixedStr(buffer);
         messageEncoding = new PrefixedStr(buffer);
         metadata = new MapStrStr(buffer);
         setComputedLength(2 * Short.BYTES + topic.getItemTotalLength() + messageEncoding.getItemTotalLength() + metadata.getItemTotalLength());
      }

      public int id()
      {
         return id;
      }

      public int schemaId()
      {
         return schemaId;
      }

      public PrefixedStr topic()
      {
         return topic;
      }

      public PrefixedStr messageEncoding()
      {
         return messageEncoding;
      }

      public MapStrStr metadata()
      {
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
         out += "\n\t-metadata = [%s]".formatted(metadata.toString());
         return out;
      }
   }

   public static class MessageIndex extends KaitaiStruct
   {
      private int channelId;
      private long lenRecords;
      private MessageIndexEntries records;

      public MessageIndex(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public MessageIndex(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         channelId = Short.toUnsignedInt(buffer.getShort());
         lenRecords = Integer.toUnsignedLong(buffer.getInt());
         records = new MessageIndexEntries(buffer, (int) lenRecords);
         setComputedLength(Short.BYTES + Integer.BYTES + records.getItemTotalLength());
      }

      public static class MessageIndexEntry extends KaitaiStruct
      {
         /**
          * Time at which the message was recorded.
          */
         private long logTime;

         /**
          * Offset is relative to the start of the uncompressed chunk data.
          */
         private long offset;

         public MessageIndexEntry(ByteBuffer buffer) throws IOException
         {
            super(buffer);
            _read();
         }

         @Override
         public void _read() throws IOException
         {
            _readIntoBuffer();
            logTime = buffer.getLong();
            offset = buffer.getLong();
            setComputedLength(2 * Long.BYTES);
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

      public static class MessageIndexEntries extends KaitaiStruct
      {
         private List<MessageIndexEntry> entries;

         public MessageIndexEntries(ByteBuffer buffer, int _length) throws IOException
         {
            super(buffer, _length);
            _read();
         }

         @Override
         public void _read() throws IOException
         {
            _readIntoBuffer();
            int remaining = _length;
            entries = new ArrayList<>();
            int computedLength = 0;

            while (remaining > 0)
            {
               MessageIndexEntry entry = new MessageIndexEntry(buffer);
               entries.add(entry);
               remaining -= entry.getItemTotalLength();
               computedLength += entry.getItemTotalLength();
            }
            setComputedLength(computedLength);
         }

         public List<MessageIndexEntry> entries()
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
            String out = getClass().getSimpleName();
            out += "\n\t-entries = " + (entries == null ? "null" : "\n" + EuclidCoreIOTools.getCollectionString("\n", entries, e -> e.toString(indent + 1)));
            return indent(out, indent);
         }
      }

      public int channelId()
      {
         return channelId;
      }

      public long lenRecords()
      {
         return lenRecords;
      }

      public MessageIndexEntries records()
      {
         return records;
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
         out += "\n\t-lenRecords = " + lenRecords;
         out += "\n\t-records = " + (records == null ? "null" : "\n" + records.toString(indent + 1));
         return indent(out, indent);
      }
   }

   public static class Statistics extends KaitaiStruct
   {
      private long messageCount;
      private int schemaCount;
      private long channelCount;
      private long attachmentCount;
      private long metadataCount;
      private long chunkCount;
      private long messageStartTime;
      private long messageEndTime;
      private long lenChannelMessageCounts;
      private ChannelMessageCounts channelMessageCounts;

      public Statistics(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Statistics(FileChannel fileChannel, long _pos, int length) throws IOException
      {
         super(fileChannel, _pos, length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         messageCount = buffer.getLong();
         schemaCount = Short.toUnsignedInt(buffer.getShort());
         channelCount = Integer.toUnsignedLong(buffer.getInt());
         attachmentCount = Integer.toUnsignedLong(buffer.getInt());
         metadataCount = Integer.toUnsignedLong(buffer.getInt());
         chunkCount = Integer.toUnsignedLong(buffer.getInt());
         messageStartTime = buffer.getLong();
         messageEndTime = buffer.getLong();
         lenChannelMessageCounts = Integer.toUnsignedLong(buffer.getInt());
         channelMessageCounts = new ChannelMessageCounts(buffer, (int) lenChannelMessageCounts);
         setComputedLength(3 * Long.BYTES + 5 * Integer.BYTES + Short.BYTES + channelMessageCounts.getItemTotalLength());
      }

      public static class ChannelMessageCounts extends KaitaiStruct
      {
         private List<ChannelMessageCount> entries;

         public ChannelMessageCounts(ByteBuffer buffer, int _length) throws IOException
         {
            super(buffer, _length);
            _read();
         }

         @Override
         public void _read() throws IOException
         {
            _readIntoBuffer();

            int remaining = _length;

            entries = new ArrayList<>();

            while (remaining > 0)
            {
               ChannelMessageCount entry = new ChannelMessageCount(buffer);
               entries.add(entry);
               remaining -= entry.getItemTotalLength();
            }
         }

         public List<ChannelMessageCount> entries()
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
            //            out += "\n\t-entries = " + (entries == null ? "null" : "\n" + EuclidCoreIOTools.getCollectionString("\n", entries, e -> e.toString(indent + 1)));
            return indent(out, indent);
         }
      }

      public static class ChannelMessageCount extends KaitaiStruct
      {
         private int channelId;
         private long messageCount;

         public ChannelMessageCount(ByteBuffer buffer) throws IOException
         {
            super(buffer);
            _read();
         }

         @Override
         public void _read() throws IOException
         {
            _readIntoBuffer();
            channelId = Short.toUnsignedInt(buffer.getShort());
            messageCount = buffer.getLong();
            setComputedLength(Short.BYTES + Long.BYTES);
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

      public long lenChannelMessageCounts()
      {
         return lenChannelMessageCounts;
      }

      public ChannelMessageCounts channelMessageCounts()
      {
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
         out += "\n\t-lenChannelMessageCounts = " + lenChannelMessageCounts;
         out += "\n\t-channelMessageCounts = \n" + channelMessageCounts.toString(1);
         return out;
      }
   }

   public static class AttachmentIndex extends KaitaiStruct
   {
      private long ofsAttachment;
      private long lenAttachment;
      private long logTime;
      private long createTime;
      private long dataSize;
      private PrefixedStr name;
      private PrefixedStr mediaType;

      private Record attachment;

      public AttachmentIndex(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public AttachmentIndex(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         ofsAttachment = buffer.getLong();
         lenAttachment = buffer.getLong();
         logTime = buffer.getLong();
         createTime = buffer.getLong();
         dataSize = buffer.getLong();
         name = new PrefixedStr(buffer);
         mediaType = new PrefixedStr(buffer);
         setComputedLength(5 * Long.BYTES + name.getItemTotalLength() + mediaType.getItemTotalLength());
      }

      public Record attachment() throws IOException
      {
         if (attachment == null)
         {
            // TODO Check if we can use the lenAttachment for verification or something.
            attachment = new Record(fileChannel, ofsAttachment);
         }

         return attachment;
      }

      public long ofsAttachment()
      {
         return ofsAttachment;
      }

      public long lenAttachment()
      {
         return lenAttachment;
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

      public PrefixedStr name()
      {
         return name;
      }

      public PrefixedStr mediaType()
      {
         return mediaType;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-ofsAttachment = " + ofsAttachment;
         out += "\n\t-lenAttachment = " + lenAttachment;
         out += "\n\t-logTime = " + logTime;
         out += "\n\t-createTime = " + createTime;
         out += "\n\t-dataSize = " + dataSize;
         out += "\n\t-name = " + name;
         out += "\n\t-mediaType = " + mediaType;
         return out;
      }
   }

   public static class Schema extends KaitaiStruct
   {
      private int id;
      private PrefixedStr name;
      private PrefixedStr encoding;
      private long lenData;
      private byte[] data;

      public Schema(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Schema(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         id = Short.toUnsignedInt(buffer.getShort());
         name = new PrefixedStr(buffer);
         encoding = new PrefixedStr(buffer);
         lenData = Integer.toUnsignedLong(buffer.getInt());
         // TODO See if we can skip creating an array.
         data = new byte[(int) lenData];
         buffer.get(data);
         setComputedLength(Short.BYTES + Integer.BYTES + name.getItemTotalLength() + encoding.getItemTotalLength() + (int) lenData);
      }

      public int id()
      {
         return id;
      }

      public PrefixedStr name()
      {
         return name;
      }

      public PrefixedStr encoding()
      {
         return encoding;
      }

      public long lenData()
      {
         return lenData;
      }

      public byte[] data()
      {
         return data;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-id = " + id;
         out += "\n\t-name = " + name;
         out += "\n\t-encoding = " + encoding;
         out += "\n\t-lenData = " + lenData;
         out += "\n\t-data = " + Arrays.toString(data);
         return out;
      }
   }

   public static class MapStrStr extends KaitaiStruct
   {
      private long lenEntries;
      private Entries entries;

      public MapStrStr(ByteBuffer buffer) throws IOException
      {
         super(buffer);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         lenEntries = Integer.toUnsignedLong(buffer.getInt());
         entries = new Entries(buffer, (int) lenEntries);
         setComputedLength(Integer.BYTES + entries.getItemTotalLength());
      }

      public static class Entries extends KaitaiStruct
      {
         private List<TupleStrStr> entries;

         public Entries(ByteBuffer buffer, int _length) throws IOException
         {
            super(buffer, _length);
            _read();
         }

         @Override
         public void _read() throws IOException
         {
            _readIntoBuffer();

            int remaining = _length;
            entries = new ArrayList<>();

            while (remaining > 0)
            {
               TupleStrStr entry = new TupleStrStr(buffer);
               entries.add(entry);
               remaining -= entry.getItemTotalLength();
            }
         }

         public List<TupleStrStr> entries()
         {
            return entries;
         }

         @Override
         public String toString()
         {
            if (entries == null)
               return "null";
            return EuclidCoreIOTools.getCollectionString(", ", entries, e -> "(%s)".formatted(Objects.toString(e)));
         }

         public String toKeysString()
         {
            return EuclidCoreIOTools.getCollectionString(", ", entries, e -> e.key().str());
         }
      }

      public long lenEntries()
      {
         return lenEntries;
      }

      public Entries entries()
      {
         return entries;
      }

      @Override
      public String toString()
      {
         return Objects.toString(entries);
      }

      public String toKeysString()
      {
         return entries.toKeysString();
      }
   }

   public static class SummaryOffset extends KaitaiStruct
   {
      public SummaryOffset(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public SummaryOffset(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         groupOpcode = Opcode.byId(Byte.toUnsignedInt(buffer.get()));
         ofsGroup = buffer.getLong();
         lenGroup = buffer.getLong();
         setComputedLength(Byte.BYTES + 2 * Long.BYTES);
      }

      private Records group;

      public Records group() throws IOException
      {
         if (group == null)
         {
            group = new Records(fileChannel, ofsGroup, (int) lenGroup);
         }
         return group;
      }

      private Opcode groupOpcode;
      private long ofsGroup;
      private long lenGroup;

      public Opcode groupOpcode()
      {
         return groupOpcode;
      }

      public long ofsGroup()
      {
         return ofsGroup;
      }

      public long lenGroup()
      {
         return lenGroup;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-groupOpcode = " + groupOpcode;
         out += "\n\t-ofsGroup = " + ofsGroup;
         out += "\n\t-lenGroup = " + lenGroup;
         return out;
      }
   }

   public static class Attachment extends KaitaiStruct
   {
      private long logTime;
      private long createTime;
      private PrefixedStr name;
      private PrefixedStr mediaType;
      private long lenData;
      private byte[] data;
      private long crc32;

      public Attachment(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Attachment(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         logTime = buffer.getLong();
         createTime = buffer.getLong();
         name = new PrefixedStr(buffer);
         mediaType = new PrefixedStr(buffer);
         lenData = buffer.getLong();
         // TODO See if we can skip creating an array.
         data = new byte[(int) lenData()];
         buffer.get(data);
         crc32InputEnd = buffer.position();
         crc32 = Integer.toUnsignedLong(buffer.getInt());
         setComputedLength(3 * Long.BYTES + name.getItemTotalLength() + mediaType.getItemTotalLength() + (int) lenData + Integer.BYTES);
      }

      private int crc32InputEnd;

      public int crc32InputEnd()
      {
         return crc32InputEnd;
      }

      private byte[] crc32Input;

      public byte[] crc32Input()
      {
         if (crc32Input == null)
         {
            int _pos = buffer.position();
            buffer.position(0);
            crc32Input = new byte[crc32InputEnd()];
            buffer.get(crc32Input);
            buffer.position(_pos);
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

      public PrefixedStr name()
      {
         return name;
      }

      public PrefixedStr mediaType()
      {
         return mediaType;
      }

      public long lenData()
      {
         return lenData;
      }

      public byte[] data()
      {
         return data;
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
         out += "\n\t-lenData = " + lenData;
         //         out += "\n\t-data = " + data;
         out += "\n\t-crc32 = " + crc32;
         return out;
      }
   }

   public static class Metadata extends KaitaiStruct
   {
      private PrefixedStr name;
      private MapStrStr metadata;

      public Metadata(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Metadata(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         name = new PrefixedStr(buffer);
         metadata = new MapStrStr(buffer);
         setComputedLength(name.getItemTotalLength() + metadata.getItemTotalLength());
      }

      public PrefixedStr name()
      {
         return name;
      }

      public MapStrStr metadata()
      {
         return metadata;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-name = " + name;
         out += "\n\t-metadata = " + metadata.toKeysString();
         return out;
      }
   }

   public static class Header extends KaitaiStruct
   {
      private PrefixedStr profile;
      private PrefixedStr library;

      public Header(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Header(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         profile = new PrefixedStr(buffer);
         library = new PrefixedStr(buffer);
         setComputedLength(profile.getItemTotalLength() + library.getItemTotalLength());
      }

      public PrefixedStr profile()
      {
         return profile;
      }

      public PrefixedStr library()
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

   public static class Message extends KaitaiStruct
   {
      private int channelId;
      private long sequence;
      private long logTime;
      private long publishTime;
      private int offsetData;
      private int lengthData;
      private byte[] data;

      public static Message createSpoofMessageForTesting(int channelId, byte[] data)
      {
         Message message = new Message();
         message.channelId = channelId;
         message.data = data;
         return message;
      }

      private Message()
      {
         super((FileChannel) null, -1, -1);
      }

      public Message(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Message(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         channelId = Short.toUnsignedInt(buffer.getShort());
         sequence = Integer.toUnsignedLong(buffer.getInt());
         logTime = buffer.getLong();
         publishTime = buffer.getLong();
         offsetData = buffer.position();
         lengthData = _length - (Short.BYTES + Integer.BYTES + 2 * Long.BYTES);
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
       * Returns the offset of the data portion of this message in the buffer returned by {@link #messageBuffer()}.
       *
       * @return the offset of the data portion of this message.
       */
      public int offsetData()
      {
         return offsetData;
      }

      /**
       * Returns the length of the data portion of this message.
       *
       * @return the length of the data portion of this message.
       */
      public int lengthData()
      {
         return lengthData;
      }

      /**
       * Returns the buffer containing this message, the data AND the header. Use {@link #offsetData()} and {@link #lengthData()} to get the data portion.
       *
       * @return the buffer containing this message.
       */
      public ByteBuffer messageBuffer()
      {
         return buffer;
      }

      public byte[] data()
      {
         if (data == null)
         {
            data = new byte[lengthData];
            buffer.limit(offsetData + lengthData);
            buffer.position(offsetData);
            buffer.get(data);
         }
         return data;
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

   public static class TupleStrStr extends KaitaiStruct
   {
      private PrefixedStr key;
      private PrefixedStr value;

      public TupleStrStr(ByteBuffer buffer) throws IOException
      {
         super(buffer);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         key = new PrefixedStr(buffer);
         value = new PrefixedStr(buffer);
         _length = key.getItemTotalLength() + value.getItemTotalLength();
      }

      public PrefixedStr key()
      {
         return key;
      }

      public PrefixedStr value()
      {
         return value;
      }

      @Override
      public String toString()
      {
         return (key.str() + ": " + value.str()).replace("\n", "");
      }
   }

   public static class MetadataIndex extends KaitaiStruct
   {
      private long ofsMetadata;
      private long lenMetadata;
      private PrefixedStr name;
      private Record metadata;

      public MetadataIndex(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public MetadataIndex(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         ofsMetadata = buffer.getLong();
         lenMetadata = buffer.getLong();
         name = new PrefixedStr(buffer);
         setComputedLength(2 * Long.BYTES + name.getItemTotalLength());
      }

      public Record metadata() throws IOException
      {
         if (metadata == null)
         {
            // TODO Check if we can use the lenMetadata for verification or something.
            metadata = new Record(fileChannel, ofsMetadata);
         }
         return metadata;
      }

      public long ofsMetadata()
      {
         return ofsMetadata;
      }

      public long lenMetadata()
      {
         return lenMetadata;
      }

      public PrefixedStr name()
      {
         return name;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-ofsMetadata = " + ofsMetadata;
         out += "\n\t-lenMetadata = " + lenMetadata;
         out += "\n\t-name = " + name;
         return out;
      }
   }

   public static class Magic extends KaitaiStruct
   {
      public static final int MAGIC_SIZE = 8;
      public static final byte[] MAGIC_BYTES = {-119, 77, 67, 65, 80, 48, 13, 10};

      private byte[] magic;

      public Magic(FileChannel fileChannel, long _pos) throws IOException
      {
         super(fileChannel, _pos, MAGIC_SIZE);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         magic = buffer.array();
         if (!(Arrays.equals(magic(), MAGIC_BYTES)))
         {
            throw new ValidationNotEqualError(MAGIC_BYTES, magic(), buffer);
         }
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

   public static class Records extends KaitaiStruct
   {
      private List<Record> records;
      private final int totalRecordLength;

      public Records(ByteBuffer buffer) throws IOException
      {
         this(buffer, buffer.remaining());
      }

      public Records(ByteBuffer buffer, int _length) throws IOException
      {
         this(buffer, buffer.position(), _length);
      }

      public Records(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         totalRecordLength = _length;
         _read();
      }

      public Records(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, -1); // Setting the super._length to -1 to avoid creating a buffer.
         totalRecordLength = _length;
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         records = new ArrayList<>();

         int remaining = totalRecordLength;

         if (fileChannel != null)
         {
            long currentPos = _pos;
            while (remaining > 0)
            {
               Record record = new Record(fileChannel, currentPos);
               records.add(record);
               currentPos += record.getItemTotalLength();
               remaining -= record.getItemTotalLength();
            }
         }
         else
         {
            long currentPos = buffer.position();

            while (remaining > 0)
            {
               Record record = new Record(buffer);
               records.add(record);
               currentPos += record.getItemTotalLength();
               remaining -= record.getItemTotalLength();
               buffer.position((int) currentPos);
            }
         }
      }

      public List<Record> records()
      {
         return records;
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
         out += "\n\t-records = " + (records == null ? "null" : "\n" + EuclidCoreIOTools.getCollectionString("\n", records, r -> r.toString(indent + 1)));
         return indent(out, indent);
      }
   }

   public static class Footer extends KaitaiStruct
   {
      private long ofsSummarySection;
      private long ofsSummaryOffsetSection;
      private long summaryCrc32;
      private Integer ofsSummaryCrc32Input;
      private Records summaryOffsetSection;
      private Records summarySection;

      public Footer(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public Footer(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         ofsSummarySection = buffer.getLong();
         ofsSummaryOffsetSection = buffer.getLong();
         summaryCrc32 = Integer.toUnsignedLong(buffer.getInt());
         setComputedLength(2 * Long.BYTES + Integer.BYTES);
      }

      public Records summarySection() throws IOException
      {
         if (summarySection == null && ofsSummarySection != 0)
         {
            long length = ((ofsSummaryOffsetSection != 0 ? ofsSummaryOffsetSection : computeOffsetFooter(fileChannel)) - ofsSummarySection);
            summarySection = new Records(fileChannel, ofsSummarySection, (int) length);
         }
         return summarySection;
      }

      public Records summaryOffsetSection() throws IOException
      {
         if (summaryOffsetSection == null && ofsSummaryOffsetSection != 0)
         {
            summaryOffsetSection = new Records(fileChannel, ofsSummaryOffsetSection, (int) (computeOffsetFooter(fileChannel) - ofsSummaryOffsetSection));
         }
         return summaryOffsetSection;
      }

      public Integer ofsSummaryCrc32Input() throws IOException
      {
         if (ofsSummaryCrc32Input == null)
         {
            ofsSummaryCrc32Input = (int) ((ofsSummarySection() != 0 ? ofsSummarySection() : computeOffsetFooter(fileChannel)));
         }
         return ofsSummaryCrc32Input;
      }

      private byte[] summaryCrc32Input;

      public byte[] summaryCrc32Input() throws IOException
      {
         if (summaryCrc32Input == null)
         {
            ByteBuffer tmpBuffer = ByteBuffer.allocate((int) (fileChannel.size() - ofsSummaryCrc32Input() - 8 - 4));
            fileChannel.position(ofsSummaryCrc32Input());
            fileChannel.read(tmpBuffer);
            summaryCrc32Input = tmpBuffer.array();
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

   public static class Record extends KaitaiStruct
   {
      public static final int RECORD_HEADER_LENGTH = 9;
      private Opcode op;
      private long lengthBody;
      private long bodyPos;
      private Object body;

      public Record(ByteBuffer buffer) throws IOException
      {
         this(buffer, buffer.position());
      }

      public Record(ByteBuffer buffer, long _pos) throws IOException
      {
         super(buffer, _pos, -1);
         _read();
      }

      public Record(FileChannel fileChannel, long _pos) throws IOException
      {
         super(fileChannel, _pos, -1);
         // We don't want to create the buffer for the whole record, just for the info in the header.
         createBuffer(RECORD_HEADER_LENGTH);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         op = Opcode.byId(Byte.toUnsignedInt(buffer.get()));
         lengthBody = buffer.getLong();
         if (fileChannel != null)
            bodyPos = _pos + RECORD_HEADER_LENGTH;
         else
            bodyPos = buffer.position();
         setComputedLength(RECORD_HEADER_LENGTH + (int) lengthBody);
      }

      public void readBody() throws IOException
      {
         if (body != null)
            return;

         if (op == null)
         {
            if (fileChannel != null)
            {
               ByteBuffer bb = ByteBuffer.allocate((int) lengthBody);
               fileChannel.read(bb, bodyPos);
               body = bb.array();
            }
            else
            {
               body = new byte[(int) lengthBody];
               buffer.get((int) bodyPos, (byte[]) body);
            }
            return;
         }

         if (fileChannel != null)
         {
            body = switch (op)
            {
               case MESSAGE -> new Message(fileChannel, bodyPos, (int) lengthBody);
               case METADATA_INDEX -> new MetadataIndex(fileChannel, bodyPos, (int) lengthBody);
               case CHUNK -> new Chunk(fileChannel, bodyPos, (int) lengthBody);
               case SCHEMA -> new Schema(fileChannel, bodyPos, (int) lengthBody);
               case CHUNK_INDEX -> new ChunkIndex(fileChannel, bodyPos, (int) lengthBody);
               case DATA_END -> new DataEnd(fileChannel, bodyPos, (int) lengthBody);
               case ATTACHMENT_INDEX -> new AttachmentIndex(fileChannel, bodyPos, (int) lengthBody);
               case STATISTICS -> new Statistics(fileChannel, bodyPos, (int) lengthBody);
               case MESSAGE_INDEX -> new MessageIndex(fileChannel, bodyPos, (int) lengthBody);
               case CHANNEL -> new Channel(fileChannel, bodyPos, (int) lengthBody);
               case METADATA -> new Metadata(fileChannel, bodyPos, (int) lengthBody);
               case ATTACHMENT -> new Attachment(fileChannel, bodyPos, (int) lengthBody);
               case HEADER -> new Header(fileChannel, bodyPos, (int) lengthBody);
               case FOOTER -> new Footer(fileChannel, bodyPos, (int) lengthBody);
               case SUMMARY_OFFSET -> new SummaryOffset(fileChannel, bodyPos, (int) lengthBody);
            };
         }
         else
         {
            body = switch (op)
            {
               case MESSAGE -> new Message(buffer, bodyPos, (int) lengthBody);
               case METADATA_INDEX -> new MetadataIndex(buffer, bodyPos, (int) lengthBody);
               case CHUNK -> new Chunk(buffer, bodyPos, (int) lengthBody);
               case SCHEMA -> new Schema(buffer, bodyPos, (int) lengthBody);
               case CHUNK_INDEX -> new ChunkIndex(buffer, bodyPos, (int) lengthBody);
               case DATA_END -> new DataEnd(buffer, bodyPos, (int) lengthBody);
               case ATTACHMENT_INDEX -> new AttachmentIndex(buffer, bodyPos, (int) lengthBody);
               case STATISTICS -> new Statistics(buffer, bodyPos, (int) lengthBody);
               case MESSAGE_INDEX -> new MessageIndex(buffer, bodyPos, (int) lengthBody);
               case CHANNEL -> new Channel(buffer, bodyPos, (int) lengthBody);
               case METADATA -> new Metadata(buffer, bodyPos, (int) lengthBody);
               case ATTACHMENT -> new Attachment(buffer, bodyPos, (int) lengthBody);
               case HEADER -> new Header(buffer, bodyPos, (int) lengthBody);
               case FOOTER -> new Footer(buffer, bodyPos, (int) lengthBody);
               case SUMMARY_OFFSET -> new SummaryOffset(buffer, bodyPos, (int) lengthBody);
            };
         }

         setComputedLength(RECORD_HEADER_LENGTH + ((KaitaiStruct) body).getItemTotalLength());
      }

      public Opcode op()
      {
         return op;
      }

      public long lengthBody()
      {
         return lengthBody;
      }

      public Object body()
      {
         try
         {
            readBody();
            return body;
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }

      public void unloadBody()
      {
         body = null;
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
         out += "\n\t-lengthBody = " + lengthBody;
         out += "\n\t-body = " + (body == null ? "null" : "\n" + ((KaitaiStruct) body).toString(indent + 2));
         return indent(out, indent);
      }
   }

   public static class ChunkIndex extends KaitaiStruct
   {
      /**
       * Earliest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private long messageStartTime;
      /**
       * Latest message log_time in the chunk. Zero if the chunk has no messages.
       */
      private long messageEndTime;
      /**
       * Offset to the chunk record from the start of the file.
       */
      private long ofsChunk;
      /**
       * Byte length of the chunk record, including opcode and length prefix.
       */
      private long lenChunk;
      /**
       * Total length in bytes of the message index records after the chunk.
       */
      private long lenMessageIndexOffsets;
      /**
       * Mapping from channel ID to the offset of the message index record for that channel after the chunk, from the start of the file. An empty map indicates
       * no message indexing is available.
       */
      private MessageIndexOffsets messageIndexOffsets;
      /**
       * Total length in bytes of the message index records after the chunk.
       */
      private long messageIndexLength;
      /**
       * The compression used within the chunk. Refer to well-known compression formats. This field should match the the value in the corresponding Chunk
       * record.
       */
      private PrefixedStr compression;
      /**
       * The size of the chunk records field.
       */
      private long compressedSize;
      /**
       * The uncompressed size of the chunk records field. This field should match the value in the corresponding Chunk record.
       */
      private long uncompressedSize;

      public ChunkIndex(ByteBuffer buffer, long _pos, int _length) throws IOException
      {
         super(buffer, _pos, _length);
         _read();
      }

      public ChunkIndex(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         super(fileChannel, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         messageStartTime = buffer.getLong();
         messageEndTime = buffer.getLong();
         ofsChunk = buffer.getLong();
         lenChunk = buffer.getLong();
         lenMessageIndexOffsets = Integer.toUnsignedLong(buffer.getInt());
         messageIndexOffsets = new MessageIndexOffsets(buffer, (int) lenMessageIndexOffsets);
         messageIndexLength = buffer.getLong();
         compression = new PrefixedStr(buffer);
         compressedSize = buffer.getLong();
         uncompressedSize = buffer.getLong();
         setComputedLength(7 * Long.BYTES + Integer.BYTES + messageIndexOffsets.getItemTotalLength() + compression.getItemTotalLength());
      }

      public static class MessageIndexOffset extends KaitaiStruct
      {
         /**
          * Channel ID.
          */
         private int channelId;
         /**
          * Offset of the message index record for that channel after the chunk, from the start of the file.
          */
         private long offset;

         public MessageIndexOffset(ByteBuffer buffer) throws IOException
         {
            super(buffer);
            _read();
         }

         @Override
         public void _read() throws IOException
         {
            _readIntoBuffer();
            channelId = Short.toUnsignedInt(buffer.getShort());
            offset = buffer.getLong();
            setComputedLength(Short.BYTES + Long.BYTES);
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

      public static class MessageIndexOffsets extends KaitaiStruct
      {
         private List<MessageIndexOffset> entries;

         public MessageIndexOffsets(ByteBuffer buffer, int _length) throws IOException
         {
            super(buffer, _length);
            _read();
         }

         @Override
         public void _read() throws IOException
         {
            _readIntoBuffer();
            entries = new ArrayList<>();

            int remaining = _length;

            while (remaining > 0)
            {
               MessageIndexOffset entry = new MessageIndexOffset(buffer);
               entries.add(entry);
               remaining -= entry.getItemTotalLength();
            }
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

      private Record chunk;

      public Record chunk() throws IOException
      {
         if (chunk == null)
         {
            // TODO Check if we can use the lenChunk for verification or something.
            chunk = new Record(fileChannel, ofsChunk);
         }
         return chunk;
      }

      public void unloadChunk()
      {
         if (chunk != null)
            chunk.unloadBody();
         chunk = null;
      }

      public long messageStartTime()
      {
         return messageStartTime;
      }

      public long messageEndTime()
      {
         return messageEndTime;
      }

      public long ofsChunk()
      {
         return ofsChunk;
      }

      public long lenChunk()
      {
         return lenChunk;
      }

      public long lenMessageIndexOffsets()
      {
         return lenMessageIndexOffsets;
      }

      public MessageIndexOffsets messageIndexOffsets()
      {
         return messageIndexOffsets;
      }

      public long messageIndexLength()
      {
         return messageIndexLength;
      }

      public PrefixedStr compression()
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
         out += "\n\t-ofsChunk = " + ofsChunk;
         out += "\n\t-lenChunk = " + lenChunk;
         out += "\n\t-lenMessageIndexOffsets = " + lenMessageIndexOffsets;
         out += "\n\t-messageIndexOffsets = " + (messageIndexOffsets == null ? "null" : "\n" + messageIndexOffsets.toString(indent + 1));
         out += "\n\t-messageIndexLength = " + messageIndexLength;
         out += "\n\t-compression = " + compression;
         out += "\n\t-compressedSize = " + compressedSize;
         out += "\n\t-uncompressedSize = " + uncompressedSize;
         return indent(out, indent);
      }
   }

   private abstract static class KaitaiStruct
   {
      protected final FileChannel fileChannel;
      protected long _pos;
      protected int _length;

      protected ByteBuffer buffer;

      public KaitaiStruct(ByteBuffer buffer)
      {
         this(buffer, -1);
      }

      public KaitaiStruct(ByteBuffer buffer, int _length)
      {
         this(buffer, buffer.position(), _length);
      }

      public KaitaiStruct(ByteBuffer buffer, long _pos, int _length)
      {
         this.buffer = buffer;
         fileChannel = null;
         this._pos = _pos;
         this._length = _length;
         buffer.order(ByteOrder.LITTLE_ENDIAN);
      }

      public KaitaiStruct(FileChannel fileChannel, long _pos, int _length)
      {
         this.fileChannel = fileChannel;
         this._pos = _pos;
         this._length = _length;
         createBuffer(_length);
      }

      protected void createBuffer(int _length)
      {
         if (_length == -1)
         {
            buffer = null;
         }
         else
         {
            buffer = ByteBuffer.allocate(_length);
            buffer.order(ByteOrder.LITTLE_ENDIAN);
         }
      }

      protected void _readIntoBuffer() throws IOException
      {
         if (fileChannel == null)
         {
            buffer.position((int) _pos);
         }
         else
         {
            fileChannel.position(_pos);
            fileChannel.read(buffer);
            buffer.flip();
         }
      }

      public abstract void _read() throws IOException;

      protected void setComputedLength(int computedLength)
      {
         if (_length == -1)
            _length = computedLength;
         else if (_length != computedLength)
            throw new ValidationNotEqualError(computedLength, _length, buffer);
      }

      public int getItemTotalLength()
      {
         if (_length == -1)
            throw new RuntimeException("Cannot get total length of an item with unknown length.");
         return _length;
      }

      public long getPosition()
      {
         if (_pos == -1)
            throw new RuntimeException("Cannot get position of an item with unknown position.");
         return _pos;
      }

      @Override
      public abstract String toString();

      public String toString(int indent)
      {
         return indent(toString(), indent);
      }
   }

   private Record footer;

   public Record footer() throws IOException
   {
      if (footer == null)
      {
         footer = new Record(fileChannel, computeOffsetFooter(fileChannel));
      }
      return footer;
   }

   public static int computeOffsetFooter(FileChannel fileChannel) throws IOException
   {
      return (int) (((((fileChannel.size() - 1) - 8) - 20) - 8));
   }

   private Magic headerMagic;
   private ArrayList<Record> records;
   private Magic footerMagic;

   public Magic headerMagic()
   {
      return headerMagic;
   }

   public ArrayList<Record> records()
   {
      return records;
   }

   public Magic footerMagic()
   {
      return footerMagic;
   }

   /**
    * Common ancestor for all error originating from Kaitai Struct usage. Stores KSY source path,
    * pointing to an element supposedly guilty of an error.
    */
   public static class KaitaiStructError extends RuntimeException
   {
      @Serial
      private static final long serialVersionUID = 3448466497836212719L;

      public KaitaiStructError(String msg)
      {
         super(msg);
      }
   }

   /**
    * Common ancestor for all validation failures. Stores pointer to KaitaiStream IO object which was
    * involved in an error.
    */
   public static class ValidationFailedError extends KaitaiStructError
   {
      @Serial
      private static final long serialVersionUID = 4069741066320518907L;

      public ValidationFailedError(String msg, ByteBuffer buffer)
      {
         super("at pos " + buffer.position() + ": validation failed: " + msg);
      }

      protected static String byteArrayToHex(byte[] arr)
      {
         StringBuilder sb = new StringBuilder("[");
         for (int i = 0; i < arr.length; i++)
         {
            if (i > 0)
               sb.append(' ');
            sb.append(String.format("%02x", arr[i]));
         }
         sb.append(']');
         return sb.toString();
      }
   }

   /**
    * Signals validation failure: we required "actual" value to be equal to "expected", but it turned
    * out that it's not.
    */
   public static class ValidationNotEqualError extends ValidationFailedError
   {
      @Serial
      private static final long serialVersionUID = -6127683772774212751L;

      public ValidationNotEqualError(byte[] expected, byte[] actual, ByteBuffer io)
      {
         super("not equal, expected " + byteArrayToHex(expected) + ", but got " + byteArrayToHex(actual), io);
      }

      public ValidationNotEqualError(Object expected, Object actual, ByteBuffer io)
      {
         super("not equal, expected " + expected + ", but got " + actual, io);
      }
   }
}