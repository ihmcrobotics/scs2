package us.ihmc.scs2.session.mcap;

import java.io.IOException;
import java.io.Serial;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.github.luben.zstd.ZstdDecompressCtx;

import gnu.trove.map.hash.TLongObjectHashMap;
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

   private Magic headerMagic;
   private ArrayList<Record> records;
   private Magic footerMagic;

   private Record footer;

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

   public Record footer() throws IOException
   {
      if (footer == null)
      {
         footer = new Record(fileChannel, computeOffsetFooter(fileChannel));
      }
      return footer;
   }

   public static class Chunk extends KaitaiStruct
   {
      private LZ4FrameDecoder lz4FrameDecoder;
      private ZstdDecompressCtx zstdDecompressCtx;

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
       * CRC32 checksum of uncompressed records field. A value of zero indicates that CRC validation
       * should not be performed.
       */
      private long uncompressedCrc32;
      /**
       * compression algorithm. i.e. zstd, lz4, "". An empty string indicates no compression. Refer to
       * well-known compression formats.
       */
      private String compression;
      /**
       * Offset position of the records in either in the {@code  ByteBuffer} or {@code FileChannel},
       * depending on how this chunk was created.
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
         compression = parseString(buffer);
         lengthRecords = buffer.getLong();
         offsetRecords = buffer.position();
         buffer.position((int) (offsetRecords + lengthRecords)); // Skip the records.

         setComputedLength(3 * Long.BYTES + 2 * Integer.BYTES + compression.length() + Long.BYTES + (int) lengthRecords);
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

      public String compression()
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
            if (compression.equalsIgnoreCase(""))
            {
               records = new Records(buffer, offsetRecords, (int) lengthRecords);
            }
            else if (compression.equalsIgnoreCase("lz4"))
            {
               if (lz4FrameDecoder == null)
                  lz4FrameDecoder = new LZ4FrameDecoder();
               ByteBuffer decompressedData = ByteBuffer.allocate((int) uncompressedSize);
               lz4FrameDecoder.decode(buffer, (int) offsetRecords, (int) lengthRecords, decompressedData, 0);
               records = new Records(decompressedData);
            }
            else if (compression.equalsIgnoreCase("zstd"))
            {
               if (zstdDecompressCtx == null)
                  zstdDecompressCtx = new ZstdDecompressCtx();
               int previousPosition = buffer.position();
               int previousLimit = buffer.limit();
               buffer.limit((int) (offsetRecords + lengthRecords));
               buffer.position((int) offsetRecords);
               ByteBuffer decompressedData = zstdDecompressCtx.decompress(buffer, (int) uncompressedSize);
               buffer.position(previousPosition);
               buffer.limit(previousLimit);
               records = new Records(decompressedData);
            }
            else
            {
               throw new UnsupportedOperationException("Unsupported compression algorithm: " + compression);
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
      private String topic;
      private String messageEncoding;
      private List<TupleStrStr> metadata;

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
         topic = parseString(buffer);
         messageEncoding = parseString(buffer);
         int start = buffer.position();
         metadata = parseList(buffer, TupleStrStr::new);
         int end = buffer.position();
         int metadataLength = end - start;
         setComputedLength(2 * Short.BYTES + 2 * Integer.BYTES + topic.length() + messageEncoding.length() + metadataLength);
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
      private List<MessageIndexEntry> messageIndexEntries;

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
         int start = buffer.position();
         messageIndexEntries = parseList(buffer, MessageIndexEntry::new);
         int entriesLength = buffer.position() - start;
         setComputedLength(Short.BYTES + entriesLength);
      }

      public static class MessageIndexEntry implements Sizeable
      {
         /**
          * Time at which the message was recorded.
          */
         private long logTime;

         /**
          * Offset is relative to the start of the uncompressed chunk data.
          */
         private long offset;

         public MessageIndexEntry(ByteBuffer buffer)
         {
            logTime = buffer.getLong();
            offset = buffer.getLong();
         }

         @Override
         public int getItemTotalLength()
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

      public List<MessageIndexEntry> records()
      {
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
         out += "\n\t-messageIndexEntries = " + (messageIndexEntries == null ?
               "null" :
               "\n" + EuclidCoreIOTools.getCollectionString("\n", messageIndexEntries, e -> e.toString(indent + 1)));
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
      private List<ChannelMessageCount> channelMessageCounts;

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
         int start = buffer.position();
         channelMessageCounts = parseList(buffer, ChannelMessageCount::new);
         int channelMessageCountsLength = buffer.position() - start;
         setComputedLength(3 * Long.BYTES + 5 * Integer.BYTES + Short.BYTES + channelMessageCountsLength);
      }

      public static class ChannelMessageCount implements Sizeable
      {
         private int channelId;
         private long messageCount;

         public ChannelMessageCount(ByteBuffer buffer)
         {
            channelId = Short.toUnsignedInt(buffer.getShort());
            messageCount = buffer.getLong();
         }

         @Override
         public int getItemTotalLength()
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
         out += "\n\t-channelMessageCounts = \n" + EuclidCoreIOTools.getCollectionString("\n", channelMessageCounts, e -> e.toString(1));
         return out;
      }
   }

   public static class AttachmentIndex extends KaitaiStruct
   {
      private long offsetAttachment;
      private long lengthAttachment;
      private long logTime;
      private long createTime;
      private long dataSize;
      private String name;
      private String mediaType;

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
         offsetAttachment = buffer.getLong();
         lengthAttachment = buffer.getLong();
         logTime = buffer.getLong();
         createTime = buffer.getLong();
         dataSize = buffer.getLong();
         name = parseString(buffer);
         mediaType = parseString(buffer);
         setComputedLength(5 * Long.BYTES + 2 * Integer.BYTES + name.length() + mediaType.length());
      }

      public Record attachment() throws IOException
      {
         if (attachment == null)
         {
            // TODO Check if we can use the lenAttachment for verification or something.
            attachment = new Record(fileChannel, offsetAttachment);
         }

         return attachment;
      }

      public long offsetAttachment()
      {
         return offsetAttachment;
      }

      public long lengthAttachment()
      {
         return lengthAttachment;
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
         out += "\n\t-ofsAttachment = " + offsetAttachment;
         out += "\n\t-lenAttachment = " + lengthAttachment;
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
      private String name;
      private String encoding;
      private long lengthData;
      private long offsetData;
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
         name = parseString(buffer);
         encoding = parseString(buffer);
         lengthData = Integer.toUnsignedLong(buffer.getInt());
         offsetData = buffer.position();
         buffer.position((int) (offsetData + lengthData)); // Skip the data

         setComputedLength(Short.BYTES + 3 * Integer.BYTES + name.length() + encoding.length() + (int) lengthData);
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

      public byte[] data()
      {
         if (data == null)
         {
            data = new byte[(int) lengthData];
            buffer.position((int) offsetData);
            buffer.get(data);
         }
         return data;
      }

      public void unloadData()
      {
         data = null;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-id = " + id;
         out += "\n\t-name = " + name;
         out += "\n\t-encoding = " + encoding;
         out += "\n\t-lengthData = " + lengthData;
         out += "\n\t-data = " + Arrays.toString(data());
         unloadData();
         return out;
      }
   }

   public static class SummaryOffset extends KaitaiStruct
   {
      private Opcode groupOpcode;
      private long offsetGroup;
      private long lengthGroup;

      private Records group;

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
         offsetGroup = buffer.getLong();
         lengthGroup = buffer.getLong();
         setComputedLength(Byte.BYTES + 2 * Long.BYTES);
      }

      public Records group() throws IOException
      {
         if (group == null)
         {
            group = new Records(fileChannel, offsetGroup, (int) lengthGroup);
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

   public static class Attachment extends KaitaiStruct
   {
      private long logTime;
      private long createTime;
      private String name;
      private String mediaType;
      private long lengthData;
      private long offsetData;
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
         name = parseString(buffer);
         mediaType = parseString(buffer);
         lengthData = buffer.getLong();
         offsetData = buffer.position();
         buffer.position((int) (offsetData + lengthData));
         crc32InputEnd = buffer.position();
         crc32 = Integer.toUnsignedLong(buffer.getInt());
         setComputedLength(3 * Long.BYTES + 3 * Integer.BYTES + name.length() + mediaType.length() + (int) lengthData);
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

      public byte[] data()
      {
         if (data == null)
         {
            data = new byte[(int) lengthData];
            buffer.position((int) offsetData);
            buffer.get(data);
         }
         return data;
      }

      public void unloadData()
      {
         data = null;
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

   public static class Metadata extends KaitaiStruct
   {
      private String name;
      private List<TupleStrStr> metadata;

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
         name = parseString(buffer);
         int start = buffer.position();
         metadata = parseList(buffer, TupleStrStr::new);
         int metadataLength = buffer.position() - start;
         setComputedLength(Integer.BYTES + name.length() + metadataLength);
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

   public static class Header extends KaitaiStruct
   {
      private String profile;
      private String library;

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
         profile = parseString(buffer);
         library = parseString(buffer);
         setComputedLength(2 * Integer.BYTES + profile.length() + library.length());
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
       * Returns the offset of the data portion of this message in the buffer returned by
       * {@link #messageBuffer()}.
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
       * Returns the buffer containing this message, the data AND the header. Use {@link #offsetData()}
       * and {@link #lengthData()} to get the data portion.
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

   public static class TupleStrStr implements Sizeable
   {
      private String key;
      private String value;

      public TupleStrStr(ByteBuffer buffer)
      {
         key = parseString(buffer);
         value = parseString(buffer);
      }

      @Override
      public int getItemTotalLength()
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

   public static class MetadataIndex extends KaitaiStruct
   {
      private long offsetMetadata;
      private long lengthMetadata;
      private String name;
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
         offsetMetadata = buffer.getLong();
         lengthMetadata = buffer.getLong();
         name = parseString(buffer);
         setComputedLength(2 * Long.BYTES + Integer.BYTES + name.length());
      }

      public Record metadata() throws IOException
      {
         if (metadata == null)
         {
            // TODO Check if we can use the lenMetadata for verification or something.
            metadata = new Record(fileChannel, offsetMetadata);
         }
         return metadata;
      }

      public long offsetMetadata()
      {
         return offsetMetadata;
      }

      public long lengthMetadata()
      {
         return lengthMetadata;
      }

      public String name()
      {
         return name;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ": ";
         out += "\n\t-offsetMetadata = " + offsetMetadata;
         out += "\n\t-lengthMetadata = " + lengthMetadata;
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
         magic = new byte[MAGIC_SIZE];
         buffer.get(magic);
         if (!(Arrays.equals(magic, MAGIC_BYTES)))
         {
            throw new ValidationNotEqualError(MAGIC_BYTES, magic, buffer);
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

   public static class Records extends ArrayList<Record>
   {
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
         parseList(buffer, Record::new, _pos, _length, this);
      }

      public Records(FileChannel fileChannel, long _pos, int _length) throws IOException
      {
         parseList(fileChannel, Record::new, _pos, _length, this);
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
       * Mapping from channel ID to the offset of the message index record for that channel after the
       * chunk, from the start of the file. An empty map indicates no message indexing is available.
       */
      private MessageIndexOffsets messageIndexOffsets;
      /**
       * Total length in bytes of the message index records after the chunk.
       */
      private long messageIndexLength;
      /**
       * The compression used within the chunk. Refer to well-known compression formats. This field should
       * match the the value in the corresponding Chunk record.
       */
      private String compression;
      /**
       * The size of the chunk records field.
       */
      private long compressedSize;
      /**
       * The uncompressed size of the chunk records field. This field should match the value in the
       * corresponding Chunk record.
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
         compression = parseString(buffer);
         compressedSize = buffer.getLong();
         uncompressedSize = buffer.getLong();
         setComputedLength(7 * Long.BYTES + 2 * Integer.BYTES + messageIndexOffsets.getItemTotalLength() + compression.length());
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

   private abstract static class KaitaiStruct implements Sizeable
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
            /*
             * TODO Unclear if it is better to switch to a direct buffer or not.
             *  While it is faster to read from a direct buffer,
             *  it is slower to create/destroy it which can be a problem if we are reading a lot of small records.
             */
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

   public interface Sizeable
   {
      int getItemTotalLength();
   }

   public static int computeOffsetFooter(FileChannel fileChannel) throws IOException
   {
      return (int) (((((fileChannel.size() - 1) - 8) - 20) - 8));
   }

   /**
    * Parses a string from the buffer. The length of the string is read from the buffer as a prefixed 32-bit unsigned integer.
    *
    * @param buffer the buffer to read from.
    * @return the string read from the buffer.
    */
   public static String parseString(ByteBuffer buffer)
   {
      return parseString(buffer, Integer.toUnsignedLong(buffer.getInt()));
   }

   /**
    * Parses a string from the buffer. The length of the string is given as a parameter.
    *
    * @param buffer the buffer to read from.
    * @param length the length in bytes of the string to read.
    * @return the string read from the buffer.
    */
   public static String parseString(ByteBuffer buffer, long length)
   {
      byte[] bytes = new byte[(int) length];
      buffer.get(bytes);
      return new String(bytes, StandardCharsets.UTF_8);
   }

   /**
    * Parses a list from the buffer. The length of the list is read from the buffer as a prefixed 32-bit unsigned integer.
    *
    * @param buffer        the buffer to read from.
    * @param elementParser the function to use to parse each element of the list.
    * @param <T>           the type of the elements in the list.
    * @return the list read from the buffer.
    */
   public static <T extends Sizeable> List<T> parseList(ByteBuffer buffer, ByteBufferReader<T> elementParser) throws IOException
   {
      return parseList(buffer, elementParser, Integer.toUnsignedLong(buffer.getInt()));
   }

   /**
    * Parses a list from the buffer. The length of the list is given as a parameter.
    *
    * @param buffer        the buffer to read from.
    * @param elementParser the function to use to parse each element of the list.
    * @param length        the length in bytes of the list to read.
    * @param <T>           the type of the elements in the list.
    * @return the list read from the buffer.
    */
   public static <T extends Sizeable> List<T> parseList(ByteBuffer buffer, ByteBufferReader<T> elementParser, long length) throws IOException
   {
      return parseList(buffer, elementParser, buffer.position(), length);
   }

   /**
    * Parses a list from the buffer. The length of the list is given as a parameter.
    *
    * @param buffer        the buffer to read from.
    * @param elementParser the function to use to parse each element of the list.
    * @param offset        the offset in the buffer to start reading from.
    * @param length        the length in bytes of the list to read.
    * @param <T>           the type of the elements in the list.
    * @return the list read from the buffer.
    */
   public static <T extends Sizeable> List<T> parseList(ByteBuffer buffer, ByteBufferReader<T> elementParser, long offset, long length) throws IOException
   {
      return parseList(buffer, elementParser, offset, length, null);
   }

   public static <T extends Sizeable> List<T> parseList(ByteBuffer buffer, ByteBufferReader<T> elementParser, long offset, long length, List<T> listToPack)
         throws IOException
   {
      buffer.position((int) offset);
      int position = buffer.position();
      long limit = position + length;
      if (listToPack == null)
         listToPack = new ArrayList<>();

      while (position < limit)
      {
         buffer.position(position);
         T parsed = elementParser.parse(buffer);
         listToPack.add(parsed);
         position += parsed.getItemTotalLength();
      }

      return listToPack;
   }

   public interface ByteBufferReader<T extends Sizeable>
   {
      T parse(ByteBuffer buffer) throws IOException;
   }

   public static <T extends Sizeable> List<T> parseList(FileChannel fileChannel, FileChannelReader<T> elementParser, long offset, long length)
         throws IOException
   {
      return parseList(fileChannel, elementParser, offset, length, null);
   }

   public static <T extends Sizeable> List<T> parseList(FileChannel fileChannel,
                                                        FileChannelReader<T> elementParser,
                                                        long offset,
                                                        long length,
                                                        List<T> listToPack) throws IOException
   {
      long position = offset;
      long limit = position + length;
      if (listToPack == null)
         listToPack = new ArrayList<>();

      while (position < limit)
      {
         T parsed = elementParser.parse(fileChannel, position);
         listToPack.add(parsed);
         position += parsed.getItemTotalLength();
      }

      return listToPack;
   }

   public interface FileChannelReader<T extends Sizeable>
   {
      T parse(FileChannel fileChannel, long position) throws IOException;
   }

   private static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = "\t".repeat(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
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