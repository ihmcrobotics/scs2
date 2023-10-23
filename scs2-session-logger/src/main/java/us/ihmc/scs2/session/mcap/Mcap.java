package us.ihmc.scs2.session.mcap;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.Charset;
import java.util.*;

// This requires the project kaitai-struct-runtime, which is at github at
// https://github.com/kaitai-io/kaitai_struct_java_runtime

//This is a generated file! Please edit source .ksy file and use kaitai-struct-compiler to rebuild

import io.kaitai.struct.ByteBufferKaitaiStream;
import io.kaitai.struct.KaitaiStream;
import io.netty.buffer.ByteBuf;
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
   protected FileChannel _io;

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

   public Mcap(FileChannel _io) throws IOException
   {
      this._io = _io;
      _read();
   }

   public FileChannel _io()
   {
      return _io;
   }

   private void _read() throws IOException
   {
      long currentPos = 0;
      headerMagic = new Magic(_io, currentPos);
      currentPos += headerMagic.getItemTotalLength();
      records = new ArrayList<>();
      Record lastRecord = null;
      do
      {
         lastRecord = new Record(_io, currentPos);
         currentPos += lastRecord.getItemTotalLength();
         records.add(lastRecord);
      }
      while (!(lastRecord.op() == Opcode.FOOTER));

      footerMagic = new Magic(_io, currentPos);
   }

   private static String indent(String stringToIndent, int indent)
   {
      if (indent <= 0)
         return stringToIndent;
      String indentStr = "\t".repeat(indent);
      return indentStr + stringToIndent.replace("\n", "\n" + indentStr);
   }

   public static class PrefixedStr extends KaitaiStructToStringEnabled
   {
      private long lenStr;
      private String str;

      public PrefixedStr(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
         _read();
      }

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
         str = new String(buffer.array(), 1, (int) lenStr, Charset.forName("UTF-8"));
         setComputedLength((int) lenStr + Integer.BYTES);
      }

      @Override
      public int getItemTotalLength()
      {
         return (int) lenStr + Integer.BYTES;
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

   public static class Chunk extends KaitaiStructToStringEnabled
   {
      private long messageStartTime;
      private long messageEndTime;
      private long uncompressedSize;
      private long uncompressedCrc32;
      private PrefixedStr compression;
      private long lenRecords;
      private Object records;
      private byte[] _raw_records;

      public Chunk(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
         lenRecords = buffer.getLong();
         switch (compression.str())
         {
            case "":
            {
               records = new Records(_io, _pos + buffer.position(), (int) lenRecords);
               break;
            }
            default:
            {
               records = new byte[(int) lenRecords];
               buffer.get((byte[]) records);
               break;
            }
         }
         setComputedLength(3 * Long.BYTES + Integer.BYTES + compression.getItemTotalLength() + Long.BYTES + (int) lenRecords);
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
         return lenRecords;
      }

      public Object records()
      {
         return records;
      }

      public byte[] _raw_records()
      {
         return _raw_records;
      }

      @Override
      public String toString()
      {
         String out = getClass().getSimpleName() + ":";
         out += "\n\t-messageStartTime = " + messageStartTime;
         out += "\n\t-messageEndTime = " + messageEndTime;
         out += "\n\t-compression = " + compression;
         out += "\n\t-compressedSize = " + lenRecords;
         out += "\n\t-uncompressedSize = " + uncompressedSize;
         out += "\n\t-uncompressedCrc32 = " + uncompressedCrc32;
         return out;
      }
   }

   public static class DataEnd extends KaitaiStructToStringEnabled
   {
      private long dataSectionCrc32;

      public DataEnd(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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

   public static class Channel extends KaitaiStructToStringEnabled
   {
      private int id;
      private int schemaId;
      private PrefixedStr topic;
      private PrefixedStr messageEncoding;
      private MapStrStr metadata;

      public Channel(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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

   public static class MessageIndex extends KaitaiStructToStringEnabled
   {
      private int channelId;
      private long lenRecords;
      private MessageIndexEntries records;

      public MessageIndex(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         channelId = Short.toUnsignedInt(buffer.getShort());
         lenRecords = Integer.toUnsignedLong(buffer.getInt());
         records = new MessageIndexEntries(buffer, (int) lenRecords);
      }

      public static class MessageIndexEntry extends KaitaiStructToStringEnabled
      {
         private long logTime;
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

      public static class MessageIndexEntries extends KaitaiStructToStringEnabled
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

            while (remaining > 0)
            {
               MessageIndexEntry entry = new MessageIndexEntry(buffer);
               entries.add(entry);
               remaining -= entry.getItemTotalLength();
            }
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

   public static class Statistics extends KaitaiStructToStringEnabled
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

      public Statistics(FileChannel _io, long position, int length) throws IOException
      {
         super(_io, position, length);
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
      }

      public static class ChannelMessageCounts extends KaitaiStructToStringEnabled
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

      public static class ChannelMessageCount extends KaitaiStructToStringEnabled
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

   public static class AttachmentIndex extends KaitaiStructToStringEnabled
   {
      private long ofsAttachment;
      private long lenAttachment;
      private long logTime;
      private long createTime;
      private long dataSize;
      private PrefixedStr name;
      private PrefixedStr mediaType;

      private Record attachment;

      public AttachmentIndex(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
            attachment = new Record(_io, ofsAttachment);
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

   public static class Schema extends KaitaiStructToStringEnabled
   {
      private int id;
      private PrefixedStr name;
      private PrefixedStr encoding;
      private long lenData;
      private byte[] data;

      public Schema(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
         out += "\n\t-data = " + data;
         return out;
      }
   }

   public static class MapStrStr extends KaitaiStructToStringEnabled
   {
      public MapStrStr(ByteBuffer buffer) throws IOException
      {
         super(buffer);
         _read();
      }

      public MapStrStr(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();

         lenEntries = Integer.toUnsignedLong(buffer.getInt());
         entries = new Entries(buffer, (int) lenEntries);
         int computedLength = Integer.BYTES + entries.getItemTotalLength();
         if (_length == -1)
            _length = computedLength;
         else if (_length != computedLength)
            throw new ValidationNotEqualError(computedLength, _length, buffer, "/types/map_str_str/seq/1");
      }

      public static class Entries extends KaitaiStructToStringEnabled
      {
         private ArrayList<TupleStrStr> entries;

         public Entries(ByteBuffer buffer, int _length) throws IOException
         {
            super(buffer, _length);
            _read();
         }

         public Entries(FileChannel _io, long _pos, int _length) throws IOException
         {
            super(_io, _pos, _length);
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

         public ArrayList<TupleStrStr> entries()
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

      private long lenEntries;
      private Entries entries;
      private byte[] _raw_entries;

      public long lenEntries()
      {
         return lenEntries;
      }

      public Entries entries()
      {
         return entries;
      }

      public byte[] _raw_entries()
      {
         return _raw_entries;
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

   public static class SummaryOffset extends KaitaiStructToStringEnabled
   {
      public SummaryOffset(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
            group = new Records(_io, ofsGroup, (int) lenGroup);
         }
         return group;
      }

      private Opcode groupOpcode;
      private long ofsGroup;
      private long lenGroup;
      private byte[] _raw_group;

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

      public byte[] _raw_group()
      {
         return _raw_group;
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

   public static class Attachment extends KaitaiStructToStringEnabled
   {

      private long logTime;
      private long createTime;
      private PrefixedStr name;
      private PrefixedStr mediaType;
      private long lenData;
      private byte[] data;
      private long crc32;

      public Attachment(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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

      private long crc32InputStart;
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

   public static class Metadata extends KaitaiStructToStringEnabled
   {
      private PrefixedStr name;
      private MapStrStr metadata;

      public Metadata(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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

   public static class Header extends KaitaiStructToStringEnabled
   {
      private PrefixedStr profile;
      private PrefixedStr library;

      public Header(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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

   public static class Message extends KaitaiStructToStringEnabled
   {
      private int channelId;
      private long sequence;
      private long logTime;
      private long publishTime;
      private byte[] data;

      public Message(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
         data = new byte[buffer.remaining()];
         buffer.get(data);
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

      public byte[] data()
      {
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

   public static class TupleStrStr extends KaitaiStructToStringEnabled
   {
      private PrefixedStr key;
      private PrefixedStr value;

      public TupleStrStr(ByteBuffer buffer) throws IOException
      {
         super(buffer);
         _read();
      }

      public TupleStrStr(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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

   public static class MetadataIndex extends KaitaiStructToStringEnabled
   {
      private long ofsMetadata;
      private long lenMetadata;
      private PrefixedStr name;
      private Record metadata;

      public MetadataIndex(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
            metadata = new Record(_io, ofsMetadata);
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

   public static class Magic extends KaitaiStructToStringEnabled
   {
      public static final int MAGIC_SIZE = 8;
      public static final byte[] MAGIC_BYTES = {-119, 77, 67, 65, 80, 48, 13, 10};

      private byte[] magic;

      public Magic(FileChannel _io, long _pos) throws IOException
      {
         super(_io, _pos, MAGIC_SIZE);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         magic = buffer.array();
         if (!(Arrays.equals(magic(), MAGIC_BYTES)))
         {
            throw new ValidationNotEqualError(MAGIC_BYTES, magic(), buffer, "/types/magic/seq/0");
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
         out += "\n\t-magic = " + magic;
         return indent(out, indent);
      }
   }

   public static class Records extends KaitaiStructToStringEnabled
   {
      private List<Record> records;
      private int totalRecordLength;

      public Records(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, -1); // Setting the super._length to -1 to avoid creating a buffer.
         totalRecordLength = _length;
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         records = new ArrayList<>();

         int remaining = totalRecordLength;
         long currentPos = _pos;

         while (remaining > 0)
         {
            Record record = new Record(_io, currentPos);
            records.add(record);
            currentPos += record.getItemTotalLength();
            remaining -= record.getItemTotalLength();
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

   public static class Footer extends KaitaiStructToStringEnabled
   {

      private long ofsSummarySection;
      private long ofsSummaryOffsetSection;
      private long summaryCrc32;
      private Integer ofsSummaryCrc32Input;
      private Records summaryOffsetSection;
      private Records summarySection;

      public Footer(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
            long length = ((ofsSummaryOffsetSection != 0 ? ofsSummaryOffsetSection : computeOffsetFooter(_io)) - ofsSummarySection);
            summarySection = new Records(_io, ofsSummarySection, (int) length);
         }
         return summarySection;
      }

      public Records summaryOffsetSection() throws IOException
      {
         if (summaryOffsetSection == null && ofsSummaryOffsetSection != 0)
         {
            summaryOffsetSection = new Records(_io, ofsSummaryOffsetSection, (int) (computeOffsetFooter(_io) - ofsSummaryOffsetSection));
         }
         return summaryOffsetSection;
      }

      public Integer ofsSummaryCrc32Input() throws IOException
      {
         if (ofsSummaryCrc32Input == null)
         {
            ofsSummaryCrc32Input = (int) ((ofsSummarySection() != 0 ? ofsSummarySection() : computeOffsetFooter(_io)));
         }
         return ofsSummaryCrc32Input;
      }

      private byte[] summaryCrc32Input;

      public byte[] summaryCrc32Input() throws IOException
      {
         if (summaryCrc32Input == null)
         {
            ByteBuffer tmpBuffer = ByteBuffer.allocate((int) (_io.size() - ofsSummaryCrc32Input() - 8 - 4));
            _io.position(ofsSummaryCrc32Input());
            _io.read(tmpBuffer);
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

   public static class Record extends KaitaiStructToStringEnabled
   {
      public static final int RECORD_HEADER_LENGTH = 9;
      private Opcode op;
      private long lenBody;
      private Object body;

      public Record(FileChannel _io, long _pos) throws IOException
      {
         super(_io, _pos, RECORD_HEADER_LENGTH);
         _read();
      }

      @Override
      public void _read() throws IOException
      {
         _readIntoBuffer();
         op = Opcode.byId(Byte.toUnsignedInt(buffer.get()));
         lenBody = buffer.getLong();
         long bodyPos = _pos + _length;

         Opcode on = op();
         if (on != null)
         {
            body = switch (op())
            {
               case MESSAGE -> new Message(_io, bodyPos, (int) lenBody);
               case METADATA_INDEX -> new MetadataIndex(_io, bodyPos, (int) lenBody);
               case CHUNK -> new Chunk(_io, bodyPos, (int) lenBody);
               case SCHEMA -> new Schema(_io, bodyPos, (int) lenBody);
               case CHUNK_INDEX -> new ChunkIndex(_io, bodyPos, (int) lenBody);
               case DATA_END -> new DataEnd(_io, bodyPos, (int) lenBody);
               case ATTACHMENT_INDEX -> new AttachmentIndex(_io, bodyPos, (int) lenBody);
               case STATISTICS -> new Statistics(_io, bodyPos, (int) lenBody);
               case MESSAGE_INDEX -> new MessageIndex(_io, bodyPos, (int) lenBody);
               case CHANNEL -> new Channel(_io, bodyPos, (int) lenBody);
               case METADATA -> new Metadata(_io, bodyPos, (int) lenBody);
               case ATTACHMENT -> new Attachment(_io, bodyPos, (int) lenBody);
               case HEADER -> new Header(_io, bodyPos, (int) lenBody);
               case FOOTER -> new Footer(_io, bodyPos, (int) lenBody);
               case SUMMARY_OFFSET -> new SummaryOffset(_io, bodyPos, (int) lenBody);
               default ->
               {
                  ByteBuffer bb = ByteBuffer.allocate((int) lenBody());
                  _io.read(bb);
                  yield bb.array();
               }
            };
         }
         else
         {
            ByteBuffer bb = ByteBuffer.allocate((int) lenBody());
            _io.read(bb);
            body = bb.array();
         }
      }

      public Opcode op()
      {
         return op;
      }

      public long lenBody()
      {
         return lenBody;
      }

      public Object body()
      {
         return body;
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
         out += "\n\t-lenBody = " + lenBody;
         out += "\n\t-body = " + (body == null ? "null" : "\n" + ((KaitaiStructToStringEnabled) body).toString(indent + 2));
         return indent(out, indent);
      }
   }

   public static class ChunkIndex extends KaitaiStructToStringEnabled
   {
      public ChunkIndex(FileChannel _io, long _pos, int _length) throws IOException
      {
         super(_io, _pos, _length);
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
         messageIndexOffsets = new MessageIndexOffsets(buffer);
         messageIndexLength = buffer.getLong();
         compression = new PrefixedStr(buffer);
         compressedSize = buffer.getLong();
         uncompressedSize = buffer.getLong();
         setComputedLength(7 * Long.BYTES + Integer.BYTES + messageIndexOffsets.getItemTotalLength() + compression.getItemTotalLength());
      }

      public static class MessageIndexOffset extends KaitaiStructToStringEnabled
      {
         public MessageIndexOffset(ByteBuffer buffer)
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

         private int channelId;
         private long offset;

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

      public static class MessageIndexOffsets extends KaitaiStructToStringEnabled
      {
         public MessageIndexOffsets(KaitaiStream _io)
         {
            super(_io);
            _read();
         }

         @Override
         private void _read()
         {
            entries = new ArrayList<>();
            {
               int i = 0;
               while (!_io.isEof())
               {
                  entries.add(new MessageIndexOffset(_io));
                  i++;
               }
            }
         }

         private ArrayList<MessageIndexOffset> entries;

         public ArrayList<MessageIndexOffset> entries()
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

      public Record chunk()
      {
         if (chunk != null)
            return chunk;
         KaitaiStream io = _root()._io();
         long _pos = io.pos();
         io.seek(ofsChunk());
         _raw_chunk = io.readBytes(lenChunk());
         KaitaiStream _io__raw_chunk = new ByteBufferKaitaiStream(_raw_chunk);
         chunk = new Record(_io__raw_chunk);
         io.seek(_pos);
         return chunk;
      }

      private long messageStartTime;
      private long messageEndTime;
      private long ofsChunk;
      private long lenChunk;
      private long lenMessageIndexOffsets;
      private MessageIndexOffsets messageIndexOffsets;
      private long messageIndexLength;
      private PrefixedStr compression;
      private long compressedSize;
      private long uncompressedSize;

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

      public byte[] _raw_messageIndexOffsets()
      {
         return _raw_messageIndexOffsets;
      }

      public byte[] _raw_chunk()
      {
         return _raw_chunk;
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

   private abstract static class KaitaiStructToStringEnabled
   {
      protected final FileChannel _io;
      protected long _pos;
      protected int _length;

      protected ByteBuffer buffer;

      public KaitaiStructToStringEnabled(ByteBuffer buffer)
      {
         this(buffer, -1);
      }

      public KaitaiStructToStringEnabled(ByteBuffer buffer, int _length)
      {
         this.buffer = buffer;
         _io = null;
         _pos = 0;
         this._length = _length;
      }

      public KaitaiStructToStringEnabled(FileChannel _io, long _pos, int _length)
      {
         this._io = _io;
         this._pos = _pos;
         this._length = _length;
         createBuffer(_length);
      }

      private void createBuffer(int _length)
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
         if (_io == null)
            return;

         _io.position(_pos);
         _io.read(buffer);
      }

      public abstract void _read() throws IOException;

      protected void setComputedLength(int computedLength)
      {
         if (_length == -1)
            _length = computedLength;
         else if (_length != computedLength)
            throw new ValidationNotEqualError(computedLength, _length, buffer, "noidea");
      }

      public int getItemTotalLength()
      {
         return _length;
      }

      @Override
      public abstract String toString();

      public String toString(int indent)
      {
         return indent(toString(), indent);
      }
   }

   private Record footer;

   public Record footer()
   {
      if (footer != null)
         return footer;
      long _pos = _io.pos();
      _io.seek(computeOffsetFooter(_io));
      _raw_footer = _io.readBytesFull();
      KaitaiStream _io__raw_footer = new ByteBufferKaitaiStream(_raw_footer);
      footer = new Record(_io__raw_footer);
      _io.seek(_pos);
      return footer;
   }

   public static int computeOffsetFooter(FileChannel _io) throws IOException
   {
      return (int) (((((_io.size() - 1) - 8) - 20) - 8));
   }

   private Magic headerMagic;
   private ArrayList<Record> records;
   private Magic footerMagic;
   private byte[] _raw_footer;

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

   public byte[] _raw_footer()
   {
      return _raw_footer;
   }

   /**
    * Exception class for an error that occurs when some fixed content was expected to appear, but
    * actual data read was different.
    *
    * @deprecated Not used anymore in favour of {@code Validation*}-exceptions.
    */
   @Deprecated
   public static class UnexpectedDataError extends RuntimeException
   {
      public UnexpectedDataError(byte[] actual, byte[] expected)
      {
         super("Unexpected fixed contents: got " + byteArrayToHex(actual) + ", was waiting for " + byteArrayToHex(expected));
      }

      private static String byteArrayToHex(byte[] arr)
      {
         StringBuilder sb = new StringBuilder();
         for (int i = 0; i < arr.length; i++)
         {
            if (i > 0)
               sb.append(' ');
            sb.append(String.format("%02x", arr[i]));
         }
         return sb.toString();
      }
   }

   /**
    * Error that occurs when default endianness should be decided with a switch, but nothing matches
    * (although using endianness expression implies that there should be some positive result).
    */
   public static class UndecidedEndiannessError extends RuntimeException
   {
   }

   /**
    * Common ancestor for all error originating from Kaitai Struct usage. Stores KSY source path,
    * pointing to an element supposedly guilty of an error.
    */
   public static class KaitaiStructError extends RuntimeException
   {
      public KaitaiStructError(String msg, String srcPath)
      {
         super(srcPath + ": " + msg);
         this.srcPath = srcPath;
      }

      protected String srcPath;
   }

   /**
    * Common ancestor for all validation failures. Stores pointer to KaitaiStream IO object which was
    * involved in an error.
    */
   public static class ValidationFailedError extends KaitaiStructError
   {
      protected ByteBuffer io;

      public ValidationFailedError(String msg, ByteBuffer io, String srcPath)
      {
         super("at pos " + io.position() + ": validation failed: " + msg, srcPath);
         this.io = io;
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
      public ValidationNotEqualError(byte[] expected, byte[] actual, ByteBuffer io, String srcPath)
      {
         super("not equal, expected " + byteArrayToHex(expected) + ", but got " + byteArrayToHex(actual), io, srcPath);
      }

      public ValidationNotEqualError(Object expected, Object actual, ByteBuffer io, String srcPath)
      {
         super("not equal, expected " + expected + ", but got " + actual, io, srcPath);
      }

      protected Object expected;
      protected Object actual;
   }

   public static class ValidationLessThanError extends ValidationFailedError
   {
      public ValidationLessThanError(byte[] expected, byte[] actual, ByteBuffer io, String srcPath)
      {
         super("not in range, min " + byteArrayToHex(expected) + ", but got " + byteArrayToHex(actual), io, srcPath);
      }

      public ValidationLessThanError(Object min, Object actual, ByteBuffer io, String srcPath)
      {
         super("not in range, min " + min + ", but got " + actual, io, srcPath);
      }

      protected Object min;
      protected Object actual;
   }

   public static class ValidationGreaterThanError extends ValidationFailedError
   {
      public ValidationGreaterThanError(byte[] expected, byte[] actual, ByteBuffer io, String srcPath)
      {
         super("not in range, max " + byteArrayToHex(expected) + ", but got " + byteArrayToHex(actual), io, srcPath);
      }

      public ValidationGreaterThanError(Object max, Object actual, ByteBuffer io, String srcPath)
      {
         super("not in range, max " + max + ", but got " + actual, io, srcPath);
      }

      protected Object max;
      protected Object actual;
   }

   public static class ValidationNotAnyOfError extends ValidationFailedError
   {
      public ValidationNotAnyOfError(Object actual, ByteBuffer io, String srcPath)
      {
         super("not any of the list, got " + actual, io, srcPath);
      }

      protected Object actual;
   }

   public static class ValidationExprError extends ValidationFailedError
   {
      public ValidationExprError(Object actual, ByteBuffer io, String srcPath)
      {
         super("not matching the expression, got " + actual, io, srcPath);
      }

      protected Object actual;
   }
}
