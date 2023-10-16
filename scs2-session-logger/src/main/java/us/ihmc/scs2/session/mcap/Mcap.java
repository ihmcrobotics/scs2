package us.ihmc.scs2.session.mcap;

// This requires the project kaitai-struct-runtime, which is at github at
// https://github.com/kaitai-io/kaitai_struct_java_runtime

//This is a generated file! Please edit source .ksy file and use kaitai-struct-compiler to rebuild

import io.kaitai.struct.ByteBufferKaitaiStream;
import io.kaitai.struct.KaitaiStruct;
import io.kaitai.struct.KaitaiStream;
import java.io.IOException;
import java.util.Map;
import java.util.HashMap;
import java.util.ArrayList;
import java.nio.charset.Charset;
import java.util.Arrays;

/**
 * MCAP is a modular container format and logging library for pub/sub messages with arbitrary
 * message serialization. It is primarily intended for use in robotics applications, and works well
 * under various workloads, resource constraints, and durability requirements. Time values
 * (`log_time`, `publish_time`, `create_time`) are represented in nanoseconds since a
 * user-understood epoch (i.e. Unix epoch, robot boot time, etc.)
 * 
 * @see <a href="https://github.com/foxglove/mcap/tree/c1cc51d/docs/specification#readme">Source</a>
 */
public class Mcap extends KaitaiStruct
{
   public static Mcap fromFile(String fileName) throws IOException
   {
      return new Mcap(new ByteBufferKaitaiStream(fileName));
   }

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

      private static final Map<Long, Opcode> byId = new HashMap<Long, Opcode>(15);
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

   public Mcap(KaitaiStream _io)
   {
      this(_io, null, null);
   }

   public Mcap(KaitaiStream _io, KaitaiStruct _parent)
   {
      this(_io, _parent, null);
   }

   public Mcap(KaitaiStream _io, KaitaiStruct _parent, Mcap _root)
   {
      super(_io);
      this._parent = _parent;
      this._root = _root == null ? this : _root;
      _read();
   }

   private void _read()
   {
      this.headerMagic = new Magic(this._io, this, _root);
      this.records = new ArrayList<Record>();
      {
         Record _it;
         int i = 0;
         do
         {
            _it = new Record(this._io, this, _root);
            this.records.add(_it);
            i++;
         }
         while (!(_it.op() == Opcode.FOOTER));
      }
      this.footerMagic = new Magic(this._io, this, _root);
   }

   public static class PrefixedStr extends KaitaiStruct
   {
      public static PrefixedStr fromFile(String fileName) throws IOException
      {
         return new PrefixedStr(new ByteBufferKaitaiStream(fileName));
      }

      public PrefixedStr(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public PrefixedStr(KaitaiStream _io, KaitaiStruct _parent)
      {
         this(_io, _parent, null);
      }

      public PrefixedStr(KaitaiStream _io, KaitaiStruct _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.lenStr = this._io.readU4le();
         this.str = new String(this._io.readBytes(lenStr()), Charset.forName("UTF-8"));
      }

      private long lenStr;
      private String str;
      private Mcap _root;
      private KaitaiStruct _parent;

      public long lenStr()
      {
         return lenStr;
      }

      public String str()
      {
         return str;
      }

      public Mcap _root()
      {
         return _root;
      }

      public KaitaiStruct _parent()
      {
         return _parent;
      }
   }

   public static class Chunk extends KaitaiStruct
   {
      public static Chunk fromFile(String fileName) throws IOException
      {
         return new Chunk(new ByteBufferKaitaiStream(fileName));
      }

      public Chunk(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Chunk(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Chunk(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.messageStartTime = this._io.readU8le();
         this.messageEndTime = this._io.readU8le();
         this.uncompressedSize = this._io.readU8le();
         this.uncompressedCrc32 = this._io.readU4le();
         this.compression = new PrefixedStr(this._io, this, _root);
         this.lenRecords = this._io.readU8le();
         switch (compression().str())
         {
            case "":
            {
               this._raw_records = this._io.readBytes(lenRecords());
               KaitaiStream _io__raw_records = new ByteBufferKaitaiStream(_raw_records);
               this.records = new Records(_io__raw_records, this, _root);
               break;
            }
            default:
            {
               this.records = this._io.readBytes(lenRecords());
               break;
            }
         }
      }

      private long messageStartTime;
      private long messageEndTime;
      private long uncompressedSize;
      private long uncompressedCrc32;
      private PrefixedStr compression;
      private long lenRecords;
      private Object records;
      private Mcap _root;
      private Mcap.Record _parent;
      private byte[] _raw_records;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_records()
      {
         return _raw_records;
      }
   }

   public static class DataEnd extends KaitaiStruct
   {
      public static DataEnd fromFile(String fileName) throws IOException
      {
         return new DataEnd(new ByteBufferKaitaiStream(fileName));
      }

      public DataEnd(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public DataEnd(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public DataEnd(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.dataSectionCrc32 = this._io.readU4le();
      }

      private long dataSectionCrc32;
      private Mcap _root;
      private Mcap.Record _parent;

      /**
       * CRC-32 of all bytes in the data section. A value of 0 indicates the CRC-32 is not available.
       */
      public long dataSectionCrc32()
      {
         return dataSectionCrc32;
      }

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }
   }

   public static class Channel extends KaitaiStruct
   {
      public static Channel fromFile(String fileName) throws IOException
      {
         return new Channel(new ByteBufferKaitaiStream(fileName));
      }

      public Channel(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Channel(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Channel(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.id = this._io.readU2le();
         this.schemaId = this._io.readU2le();
         this.topic = new PrefixedStr(this._io, this, _root);
         this.messageEncoding = new PrefixedStr(this._io, this, _root);
         this.metadata = new MapStrStr(this._io, this, _root);
      }

      private int id;
      private int schemaId;
      private PrefixedStr topic;
      private PrefixedStr messageEncoding;
      private MapStrStr metadata;
      private Mcap _root;
      private Mcap.Record _parent;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }
   }

   public static class MessageIndex extends KaitaiStruct
   {
      public static MessageIndex fromFile(String fileName) throws IOException
      {
         return new MessageIndex(new ByteBufferKaitaiStream(fileName));
      }

      public MessageIndex(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public MessageIndex(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public MessageIndex(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.channelId = this._io.readU2le();
         this.lenRecords = this._io.readU4le();
         this._raw_records = this._io.readBytes(lenRecords());
         KaitaiStream _io__raw_records = new ByteBufferKaitaiStream(_raw_records);
         this.records = new MessageIndexEntries(_io__raw_records, this, _root);
      }

      public static class MessageIndexEntry extends KaitaiStruct
      {
         public static MessageIndexEntry fromFile(String fileName) throws IOException
         {
            return new MessageIndexEntry(new ByteBufferKaitaiStream(fileName));
         }

         public MessageIndexEntry(KaitaiStream _io)
         {
            this(_io, null, null);
         }

         public MessageIndexEntry(KaitaiStream _io, Mcap.MessageIndex.MessageIndexEntries _parent)
         {
            this(_io, _parent, null);
         }

         public MessageIndexEntry(KaitaiStream _io, Mcap.MessageIndex.MessageIndexEntries _parent, Mcap _root)
         {
            super(_io);
            this._parent = _parent;
            this._root = _root;
            _read();
         }

         private void _read()
         {
            this.logTime = this._io.readU8le();
            this.offset = this._io.readU8le();
         }

         private long logTime;
         private long offset;
         private Mcap _root;
         private Mcap.MessageIndex.MessageIndexEntries _parent;

         public long logTime()
         {
            return logTime;
         }

         public long offset()
         {
            return offset;
         }

         public Mcap _root()
         {
            return _root;
         }

         public Mcap.MessageIndex.MessageIndexEntries _parent()
         {
            return _parent;
         }
      }

      public static class MessageIndexEntries extends KaitaiStruct
      {
         public static MessageIndexEntries fromFile(String fileName) throws IOException
         {
            return new MessageIndexEntries(new ByteBufferKaitaiStream(fileName));
         }

         public MessageIndexEntries(KaitaiStream _io)
         {
            this(_io, null, null);
         }

         public MessageIndexEntries(KaitaiStream _io, Mcap.MessageIndex _parent)
         {
            this(_io, _parent, null);
         }

         public MessageIndexEntries(KaitaiStream _io, Mcap.MessageIndex _parent, Mcap _root)
         {
            super(_io);
            this._parent = _parent;
            this._root = _root;
            _read();
         }

         private void _read()
         {
            this.entries = new ArrayList<MessageIndexEntry>();
            {
               int i = 0;
               while (!this._io.isEof())
               {
                  this.entries.add(new MessageIndexEntry(this._io, this, _root));
                  i++;
               }
            }
         }

         private ArrayList<MessageIndexEntry> entries;
         private Mcap _root;
         private Mcap.MessageIndex _parent;

         public ArrayList<MessageIndexEntry> entries()
         {
            return entries;
         }

         public Mcap _root()
         {
            return _root;
         }

         public Mcap.MessageIndex _parent()
         {
            return _parent;
         }
      }

      private int channelId;
      private long lenRecords;
      private MessageIndexEntries records;
      private Mcap _root;
      private Mcap.Record _parent;
      private byte[] _raw_records;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_records()
      {
         return _raw_records;
      }
   }

   public static class Statistics extends KaitaiStruct
   {
      public static Statistics fromFile(String fileName) throws IOException
      {
         return new Statistics(new ByteBufferKaitaiStream(fileName));
      }

      public Statistics(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Statistics(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Statistics(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.messageCount = this._io.readU8le();
         this.schemaCount = this._io.readU2le();
         this.channelCount = this._io.readU4le();
         this.attachmentCount = this._io.readU4le();
         this.metadataCount = this._io.readU4le();
         this.chunkCount = this._io.readU4le();
         this.messageStartTime = this._io.readU8le();
         this.messageEndTime = this._io.readU8le();
         this.lenChannelMessageCounts = this._io.readU4le();
         this._raw_channelMessageCounts = this._io.readBytes(lenChannelMessageCounts());
         KaitaiStream _io__raw_channelMessageCounts = new ByteBufferKaitaiStream(_raw_channelMessageCounts);
         this.channelMessageCounts = new ChannelMessageCounts(_io__raw_channelMessageCounts, this, _root);
      }

      public static class ChannelMessageCounts extends KaitaiStruct
      {
         public static ChannelMessageCounts fromFile(String fileName) throws IOException
         {
            return new ChannelMessageCounts(new ByteBufferKaitaiStream(fileName));
         }

         public ChannelMessageCounts(KaitaiStream _io)
         {
            this(_io, null, null);
         }

         public ChannelMessageCounts(KaitaiStream _io, Mcap.Statistics _parent)
         {
            this(_io, _parent, null);
         }

         public ChannelMessageCounts(KaitaiStream _io, Mcap.Statistics _parent, Mcap _root)
         {
            super(_io);
            this._parent = _parent;
            this._root = _root;
            _read();
         }

         private void _read()
         {
            this.entries = new ArrayList<ChannelMessageCount>();
            {
               int i = 0;
               while (!this._io.isEof())
               {
                  this.entries.add(new ChannelMessageCount(this._io, this, _root));
                  i++;
               }
            }
         }

         private ArrayList<ChannelMessageCount> entries;
         private Mcap _root;
         private Mcap.Statistics _parent;

         public ArrayList<ChannelMessageCount> entries()
         {
            return entries;
         }

         public Mcap _root()
         {
            return _root;
         }

         public Mcap.Statistics _parent()
         {
            return _parent;
         }
      }

      public static class ChannelMessageCount extends KaitaiStruct
      {
         public static ChannelMessageCount fromFile(String fileName) throws IOException
         {
            return new ChannelMessageCount(new ByteBufferKaitaiStream(fileName));
         }

         public ChannelMessageCount(KaitaiStream _io)
         {
            this(_io, null, null);
         }

         public ChannelMessageCount(KaitaiStream _io, Mcap.Statistics.ChannelMessageCounts _parent)
         {
            this(_io, _parent, null);
         }

         public ChannelMessageCount(KaitaiStream _io, Mcap.Statistics.ChannelMessageCounts _parent, Mcap _root)
         {
            super(_io);
            this._parent = _parent;
            this._root = _root;
            _read();
         }

         private void _read()
         {
            this.channelId = this._io.readU2le();
            this.messageCount = this._io.readU8le();
         }

         private int channelId;
         private long messageCount;
         private Mcap _root;
         private Mcap.Statistics.ChannelMessageCounts _parent;

         public int channelId()
         {
            return channelId;
         }

         public long messageCount()
         {
            return messageCount;
         }

         public Mcap _root()
         {
            return _root;
         }

         public Mcap.Statistics.ChannelMessageCounts _parent()
         {
            return _parent;
         }
      }

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
      private Mcap _root;
      private Mcap.Record _parent;
      private byte[] _raw_channelMessageCounts;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_channelMessageCounts()
      {
         return _raw_channelMessageCounts;
      }
   }

   public static class AttachmentIndex extends KaitaiStruct
   {
      public static AttachmentIndex fromFile(String fileName) throws IOException
      {
         return new AttachmentIndex(new ByteBufferKaitaiStream(fileName));
      }

      public AttachmentIndex(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public AttachmentIndex(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public AttachmentIndex(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.ofsAttachment = this._io.readU8le();
         this.lenAttachment = this._io.readU8le();
         this.logTime = this._io.readU8le();
         this.createTime = this._io.readU8le();
         this.dataSize = this._io.readU8le();
         this.name = new PrefixedStr(this._io, this, _root);
         this.mediaType = new PrefixedStr(this._io, this, _root);
      }

      private Record attachment;

      public Record attachment()
      {
         if (this.attachment != null)
            return this.attachment;
         KaitaiStream io = _root()._io();
         long _pos = io.pos();
         io.seek(ofsAttachment());
         this._raw_attachment = io.readBytes(lenAttachment());
         KaitaiStream _io__raw_attachment = new ByteBufferKaitaiStream(_raw_attachment);
         this.attachment = new Record(_io__raw_attachment, this, _root);
         io.seek(_pos);
         return this.attachment;
      }

      private long ofsAttachment;
      private long lenAttachment;
      private long logTime;
      private long createTime;
      private long dataSize;
      private PrefixedStr name;
      private PrefixedStr mediaType;
      private Mcap _root;
      private Mcap.Record _parent;
      private byte[] _raw_attachment;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_attachment()
      {
         return _raw_attachment;
      }
   }

   public static class Schema extends KaitaiStruct
   {
      public static Schema fromFile(String fileName) throws IOException
      {
         return new Schema(new ByteBufferKaitaiStream(fileName));
      }

      public Schema(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Schema(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Schema(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.id = this._io.readU2le();
         this.name = new PrefixedStr(this._io, this, _root);
         this.encoding = new PrefixedStr(this._io, this, _root);
         this.lenData = this._io.readU4le();
         this.data = this._io.readBytes(lenData());
      }

      private int id;
      private PrefixedStr name;
      private PrefixedStr encoding;
      private long lenData;
      private byte[] data;
      private Mcap _root;
      private Mcap.Record _parent;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }
   }

   public static class MapStrStr extends KaitaiStruct
   {
      public static MapStrStr fromFile(String fileName) throws IOException
      {
         return new MapStrStr(new ByteBufferKaitaiStream(fileName));
      }

      public MapStrStr(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public MapStrStr(KaitaiStream _io, KaitaiStruct _parent)
      {
         this(_io, _parent, null);
      }

      public MapStrStr(KaitaiStream _io, KaitaiStruct _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.lenEntries = this._io.readU4le();
         this._raw_entries = this._io.readBytes(lenEntries());
         KaitaiStream _io__raw_entries = new ByteBufferKaitaiStream(_raw_entries);
         this.entries = new Entries(_io__raw_entries, this, _root);
      }

      public static class Entries extends KaitaiStruct
      {
         public static Entries fromFile(String fileName) throws IOException
         {
            return new Entries(new ByteBufferKaitaiStream(fileName));
         }

         public Entries(KaitaiStream _io)
         {
            this(_io, null, null);
         }

         public Entries(KaitaiStream _io, Mcap.MapStrStr _parent)
         {
            this(_io, _parent, null);
         }

         public Entries(KaitaiStream _io, Mcap.MapStrStr _parent, Mcap _root)
         {
            super(_io);
            this._parent = _parent;
            this._root = _root;
            _read();
         }

         private void _read()
         {
            this.entries = new ArrayList<TupleStrStr>();
            {
               int i = 0;
               while (!this._io.isEof())
               {
                  this.entries.add(new TupleStrStr(this._io, this, _root));
                  i++;
               }
            }
         }

         private ArrayList<TupleStrStr> entries;
         private Mcap _root;
         private Mcap.MapStrStr _parent;

         public ArrayList<TupleStrStr> entries()
         {
            return entries;
         }

         public Mcap _root()
         {
            return _root;
         }

         public Mcap.MapStrStr _parent()
         {
            return _parent;
         }
      }

      private long lenEntries;
      private Entries entries;
      private Mcap _root;
      private KaitaiStruct _parent;
      private byte[] _raw_entries;

      public long lenEntries()
      {
         return lenEntries;
      }

      public Entries entries()
      {
         return entries;
      }

      public Mcap _root()
      {
         return _root;
      }

      public KaitaiStruct _parent()
      {
         return _parent;
      }

      public byte[] _raw_entries()
      {
         return _raw_entries;
      }
   }

   public static class SummaryOffset extends KaitaiStruct
   {
      public static SummaryOffset fromFile(String fileName) throws IOException
      {
         return new SummaryOffset(new ByteBufferKaitaiStream(fileName));
      }

      public SummaryOffset(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public SummaryOffset(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public SummaryOffset(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.groupOpcode = Mcap.Opcode.byId(this._io.readU1());
         this.ofsGroup = this._io.readU8le();
         this.lenGroup = this._io.readU8le();
      }

      private Records group;

      public Records group()
      {
         if (this.group != null)
            return this.group;
         KaitaiStream io = _root()._io();
         long _pos = io.pos();
         io.seek(ofsGroup());
         this._raw_group = io.readBytes(lenGroup());
         KaitaiStream _io__raw_group = new ByteBufferKaitaiStream(_raw_group);
         this.group = new Records(_io__raw_group, this, _root);
         io.seek(_pos);
         return this.group;
      }

      private Opcode groupOpcode;
      private long ofsGroup;
      private long lenGroup;
      private Mcap _root;
      private Mcap.Record _parent;
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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_group()
      {
         return _raw_group;
      }
   }

   public static class Attachment extends KaitaiStruct
   {
      public static Attachment fromFile(String fileName) throws IOException
      {
         return new Attachment(new ByteBufferKaitaiStream(fileName));
      }

      public Attachment(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Attachment(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Attachment(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.logTime = this._io.readU8le();
         this.createTime = this._io.readU8le();
         this.name = new PrefixedStr(this._io, this, _root);
         this.mediaType = new PrefixedStr(this._io, this, _root);
         this.lenData = this._io.readU8le();
         this.data = this._io.readBytes(lenData());
         if (crc32InputEnd() >= 0)
         {
            this.invokeCrc32InputEnd = this._io.readBytes(0);
         }
         this.crc32 = this._io.readU4le();
      }

      private Integer crc32InputEnd;

      public Integer crc32InputEnd()
      {
         if (this.crc32InputEnd != null)
            return this.crc32InputEnd;
         int _tmp = (int) (_io().pos());
         this.crc32InputEnd = _tmp;
         return this.crc32InputEnd;
      }

      private byte[] crc32Input;

      public byte[] crc32Input()
      {
         if (this.crc32Input != null)
            return this.crc32Input;
         long _pos = this._io.pos();
         this._io.seek(0);
         this.crc32Input = this._io.readBytes(crc32InputEnd());
         this._io.seek(_pos);
         return this.crc32Input;
      }

      private long logTime;
      private long createTime;
      private PrefixedStr name;
      private PrefixedStr mediaType;
      private long lenData;
      private byte[] data;
      private byte[] invokeCrc32InputEnd;
      private long crc32;
      private Mcap _root;
      private Mcap.Record _parent;

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

      public byte[] invokeCrc32InputEnd()
      {
         return invokeCrc32InputEnd;
      }

      /**
       * CRC-32 checksum of preceding fields in the record. A value of zero indicates that CRC validation
       * should not be performed.
       */
      public long crc32()
      {
         return crc32;
      }

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }
   }

   public static class Metadata extends KaitaiStruct
   {
      public static Metadata fromFile(String fileName) throws IOException
      {
         return new Metadata(new ByteBufferKaitaiStream(fileName));
      }

      public Metadata(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Metadata(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Metadata(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.name = new PrefixedStr(this._io, this, _root);
         this.metadata = new MapStrStr(this._io, this, _root);
      }

      private PrefixedStr name;
      private MapStrStr metadata;
      private Mcap _root;
      private Mcap.Record _parent;

      public PrefixedStr name()
      {
         return name;
      }

      public MapStrStr metadata()
      {
         return metadata;
      }

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }
   }

   public static class Header extends KaitaiStruct
   {
      public static Header fromFile(String fileName) throws IOException
      {
         return new Header(new ByteBufferKaitaiStream(fileName));
      }

      public Header(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Header(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Header(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.profile = new PrefixedStr(this._io, this, _root);
         this.library = new PrefixedStr(this._io, this, _root);
      }

      private PrefixedStr profile;
      private PrefixedStr library;
      private Mcap _root;
      private Mcap.Record _parent;

      public PrefixedStr profile()
      {
         return profile;
      }

      public PrefixedStr library()
      {
         return library;
      }

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }
   }

   public static class Message extends KaitaiStruct
   {
      public static Message fromFile(String fileName) throws IOException
      {
         return new Message(new ByteBufferKaitaiStream(fileName));
      }

      public Message(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Message(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Message(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.channelId = this._io.readU2le();
         this.sequence = this._io.readU4le();
         this.logTime = this._io.readU8le();
         this.publishTime = this._io.readU8le();
         this.data = this._io.readBytesFull();
      }

      private int channelId;
      private long sequence;
      private long logTime;
      private long publishTime;
      private byte[] data;
      private Mcap _root;
      private Mcap.Record _parent;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }
   }

   public static class TupleStrStr extends KaitaiStruct
   {
      public static TupleStrStr fromFile(String fileName) throws IOException
      {
         return new TupleStrStr(new ByteBufferKaitaiStream(fileName));
      }

      public TupleStrStr(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public TupleStrStr(KaitaiStream _io, Mcap.MapStrStr.Entries _parent)
      {
         this(_io, _parent, null);
      }

      public TupleStrStr(KaitaiStream _io, Mcap.MapStrStr.Entries _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.key = new PrefixedStr(this._io, this, _root);
         this.value = new PrefixedStr(this._io, this, _root);
      }

      private PrefixedStr key;
      private PrefixedStr value;
      private Mcap _root;
      private Mcap.MapStrStr.Entries _parent;

      public PrefixedStr key()
      {
         return key;
      }

      public PrefixedStr value()
      {
         return value;
      }

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.MapStrStr.Entries _parent()
      {
         return _parent;
      }
   }

   public static class MetadataIndex extends KaitaiStruct
   {
      public static MetadataIndex fromFile(String fileName) throws IOException
      {
         return new MetadataIndex(new ByteBufferKaitaiStream(fileName));
      }

      public MetadataIndex(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public MetadataIndex(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public MetadataIndex(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.ofsMetadata = this._io.readU8le();
         this.lenMetadata = this._io.readU8le();
         this.name = new PrefixedStr(this._io, this, _root);
      }

      private Record metadata;

      public Record metadata()
      {
         if (this.metadata != null)
            return this.metadata;
         KaitaiStream io = _root()._io();
         long _pos = io.pos();
         io.seek(ofsMetadata());
         this._raw_metadata = io.readBytes(lenMetadata());
         KaitaiStream _io__raw_metadata = new ByteBufferKaitaiStream(_raw_metadata);
         this.metadata = new Record(_io__raw_metadata, this, _root);
         io.seek(_pos);
         return this.metadata;
      }

      private long ofsMetadata;
      private long lenMetadata;
      private PrefixedStr name;
      private Mcap _root;
      private Mcap.Record _parent;
      private byte[] _raw_metadata;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_metadata()
      {
         return _raw_metadata;
      }
   }

   public static class Magic extends KaitaiStruct
   {
      public static Magic fromFile(String fileName) throws IOException
      {
         return new Magic(new ByteBufferKaitaiStream(fileName));
      }

      public Magic(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Magic(KaitaiStream _io, Mcap _parent)
      {
         this(_io, _parent, null);
      }

      public Magic(KaitaiStream _io, Mcap _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.magic = this._io.readBytes(8);
         if (!(Arrays.equals(magic(), new byte[] {-119, 77, 67, 65, 80, 48, 13, 10})))
         {
            throw new KaitaiStream.ValidationNotEqualError(new byte[] {-119, 77, 67, 65, 80, 48, 13, 10}, magic(), _io(), "/types/magic/seq/0");
         }
      }

      private byte[] magic;
      private Mcap _root;
      private Mcap _parent;

      public byte[] magic()
      {
         return magic;
      }

      public Mcap _root()
      {
         return _root;
      }

      public Mcap _parent()
      {
         return _parent;
      }
   }

   public static class Records extends KaitaiStruct
   {
      public static Records fromFile(String fileName) throws IOException
      {
         return new Records(new ByteBufferKaitaiStream(fileName));
      }

      public Records(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Records(KaitaiStream _io, KaitaiStruct _parent)
      {
         this(_io, _parent, null);
      }

      public Records(KaitaiStream _io, KaitaiStruct _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.records = new ArrayList<Record>();
         {
            int i = 0;
            while (!this._io.isEof())
            {
               this.records.add(new Record(this._io, this, _root));
               i++;
            }
         }
      }

      private ArrayList<Record> records;
      private Mcap _root;
      private KaitaiStruct _parent;

      public ArrayList<Record> records()
      {
         return records;
      }

      public Mcap _root()
      {
         return _root;
      }

      public KaitaiStruct _parent()
      {
         return _parent;
      }
   }

   public static class Footer extends KaitaiStruct
   {
      public static Footer fromFile(String fileName) throws IOException
      {
         return new Footer(new ByteBufferKaitaiStream(fileName));
      }

      public Footer(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Footer(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public Footer(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.ofsSummarySection = this._io.readU8le();
         this.ofsSummaryOffsetSection = this._io.readU8le();
         this.summaryCrc32 = this._io.readU4le();
      }

      private Records summarySection;

      public Records summarySection()
      {
         if (this.summarySection != null)
            return this.summarySection;
         if (ofsSummarySection() != 0)
         {
            KaitaiStream io = _root()._io();
            long _pos = io.pos();
            io.seek(ofsSummarySection());
            this._raw_summarySection = io.readBytes(((ofsSummaryOffsetSection() != 0 ? ofsSummaryOffsetSection() : _root().ofsFooter()) - ofsSummarySection()));
            KaitaiStream _io__raw_summarySection = new ByteBufferKaitaiStream(_raw_summarySection);
            this.summarySection = new Records(_io__raw_summarySection, this, _root);
            io.seek(_pos);
         }
         return this.summarySection;
      }

      private Records summaryOffsetSection;

      public Records summaryOffsetSection()
      {
         if (this.summaryOffsetSection != null)
            return this.summaryOffsetSection;
         if (ofsSummaryOffsetSection() != 0)
         {
            KaitaiStream io = _root()._io();
            long _pos = io.pos();
            io.seek(ofsSummaryOffsetSection());
            this._raw_summaryOffsetSection = io.readBytes((_root().ofsFooter() - ofsSummaryOffsetSection()));
            KaitaiStream _io__raw_summaryOffsetSection = new ByteBufferKaitaiStream(_raw_summaryOffsetSection);
            this.summaryOffsetSection = new Records(_io__raw_summaryOffsetSection, this, _root);
            io.seek(_pos);
         }
         return this.summaryOffsetSection;
      }

      private Integer ofsSummaryCrc32Input;

      public Integer ofsSummaryCrc32Input()
      {
         if (this.ofsSummaryCrc32Input != null)
            return this.ofsSummaryCrc32Input;
         int _tmp = (int) ((ofsSummarySection() != 0 ? ofsSummarySection() : _root().ofsFooter()));
         this.ofsSummaryCrc32Input = _tmp;
         return this.ofsSummaryCrc32Input;
      }

      private byte[] summaryCrc32Input;

      public byte[] summaryCrc32Input()
      {
         if (this.summaryCrc32Input != null)
            return this.summaryCrc32Input;
         KaitaiStream io = _root()._io();
         long _pos = io.pos();
         io.seek(ofsSummaryCrc32Input());
         this.summaryCrc32Input = io.readBytes((((_root()._io().size() - ofsSummaryCrc32Input()) - 8) - 4));
         io.seek(_pos);
         return this.summaryCrc32Input;
      }

      private long ofsSummarySection;
      private long ofsSummaryOffsetSection;
      private long summaryCrc32;
      private Mcap _root;
      private Mcap.Record _parent;
      private byte[] _raw_summarySection;
      private byte[] _raw_summaryOffsetSection;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_summarySection()
      {
         return _raw_summarySection;
      }

      public byte[] _raw_summaryOffsetSection()
      {
         return _raw_summaryOffsetSection;
      }
   }

   public static class Record extends KaitaiStruct
   {
      public static Record fromFile(String fileName) throws IOException
      {
         return new Record(new ByteBufferKaitaiStream(fileName));
      }

      public Record(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public Record(KaitaiStream _io, KaitaiStruct _parent)
      {
         this(_io, _parent, null);
      }

      public Record(KaitaiStream _io, KaitaiStruct _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.op = Mcap.Opcode.byId(this._io.readU1());
         this.lenBody = this._io.readU8le();
         {
            Opcode on = op();
            if (on != null)
            {
               switch (op())
               {
                  case MESSAGE:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Message(_io__raw_body, this, _root);
                     break;
                  }
                  case METADATA_INDEX:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new MetadataIndex(_io__raw_body, this, _root);
                     break;
                  }
                  case CHUNK:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Chunk(_io__raw_body, this, _root);
                     break;
                  }
                  case SCHEMA:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Schema(_io__raw_body, this, _root);
                     break;
                  }
                  case CHUNK_INDEX:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new ChunkIndex(_io__raw_body, this, _root);
                     break;
                  }
                  case DATA_END:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new DataEnd(_io__raw_body, this, _root);
                     break;
                  }
                  case ATTACHMENT_INDEX:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new AttachmentIndex(_io__raw_body, this, _root);
                     break;
                  }
                  case STATISTICS:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Statistics(_io__raw_body, this, _root);
                     break;
                  }
                  case MESSAGE_INDEX:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new MessageIndex(_io__raw_body, this, _root);
                     break;
                  }
                  case CHANNEL:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Channel(_io__raw_body, this, _root);
                     break;
                  }
                  case METADATA:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Metadata(_io__raw_body, this, _root);
                     break;
                  }
                  case ATTACHMENT:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Attachment(_io__raw_body, this, _root);
                     break;
                  }
                  case HEADER:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Header(_io__raw_body, this, _root);
                     break;
                  }
                  case FOOTER:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new Footer(_io__raw_body, this, _root);
                     break;
                  }
                  case SUMMARY_OFFSET:
                  {
                     this._raw_body = this._io.readBytes(lenBody());
                     KaitaiStream _io__raw_body = new ByteBufferKaitaiStream(_raw_body);
                     this.body = new SummaryOffset(_io__raw_body, this, _root);
                     break;
                  }
                  default:
                  {
                     this.body = this._io.readBytes(lenBody());
                     break;
                  }
               }
            }
            else
            {
               this.body = this._io.readBytes(lenBody());
            }
         }
      }

      private Opcode op;
      private long lenBody;
      private Object body;
      private Mcap _root;
      private KaitaiStruct _parent;
      private byte[] _raw_body;

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

      public Mcap _root()
      {
         return _root;
      }

      public KaitaiStruct _parent()
      {
         return _parent;
      }

      public byte[] _raw_body()
      {
         return _raw_body;
      }
   }

   public static class ChunkIndex extends KaitaiStruct
   {
      public static ChunkIndex fromFile(String fileName) throws IOException
      {
         return new ChunkIndex(new ByteBufferKaitaiStream(fileName));
      }

      public ChunkIndex(KaitaiStream _io)
      {
         this(_io, null, null);
      }

      public ChunkIndex(KaitaiStream _io, Mcap.Record _parent)
      {
         this(_io, _parent, null);
      }

      public ChunkIndex(KaitaiStream _io, Mcap.Record _parent, Mcap _root)
      {
         super(_io);
         this._parent = _parent;
         this._root = _root;
         _read();
      }

      private void _read()
      {
         this.messageStartTime = this._io.readU8le();
         this.messageEndTime = this._io.readU8le();
         this.ofsChunk = this._io.readU8le();
         this.lenChunk = this._io.readU8le();
         this.lenMessageIndexOffsets = this._io.readU4le();
         this._raw_messageIndexOffsets = this._io.readBytes(lenMessageIndexOffsets());
         KaitaiStream _io__raw_messageIndexOffsets = new ByteBufferKaitaiStream(_raw_messageIndexOffsets);
         this.messageIndexOffsets = new MessageIndexOffsets(_io__raw_messageIndexOffsets, this, _root);
         this.messageIndexLength = this._io.readU8le();
         this.compression = new PrefixedStr(this._io, this, _root);
         this.compressedSize = this._io.readU8le();
         this.uncompressedSize = this._io.readU8le();
      }

      public static class MessageIndexOffset extends KaitaiStruct
      {
         public static MessageIndexOffset fromFile(String fileName) throws IOException
         {
            return new MessageIndexOffset(new ByteBufferKaitaiStream(fileName));
         }

         public MessageIndexOffset(KaitaiStream _io)
         {
            this(_io, null, null);
         }

         public MessageIndexOffset(KaitaiStream _io, Mcap.ChunkIndex.MessageIndexOffsets _parent)
         {
            this(_io, _parent, null);
         }

         public MessageIndexOffset(KaitaiStream _io, Mcap.ChunkIndex.MessageIndexOffsets _parent, Mcap _root)
         {
            super(_io);
            this._parent = _parent;
            this._root = _root;
            _read();
         }

         private void _read()
         {
            this.channelId = this._io.readU2le();
            this.offset = this._io.readU8le();
         }

         private int channelId;
         private long offset;
         private Mcap _root;
         private Mcap.ChunkIndex.MessageIndexOffsets _parent;

         public int channelId()
         {
            return channelId;
         }

         public long offset()
         {
            return offset;
         }

         public Mcap _root()
         {
            return _root;
         }

         public Mcap.ChunkIndex.MessageIndexOffsets _parent()
         {
            return _parent;
         }
      }

      public static class MessageIndexOffsets extends KaitaiStruct
      {
         public static MessageIndexOffsets fromFile(String fileName) throws IOException
         {
            return new MessageIndexOffsets(new ByteBufferKaitaiStream(fileName));
         }

         public MessageIndexOffsets(KaitaiStream _io)
         {
            this(_io, null, null);
         }

         public MessageIndexOffsets(KaitaiStream _io, Mcap.ChunkIndex _parent)
         {
            this(_io, _parent, null);
         }

         public MessageIndexOffsets(KaitaiStream _io, Mcap.ChunkIndex _parent, Mcap _root)
         {
            super(_io);
            this._parent = _parent;
            this._root = _root;
            _read();
         }

         private void _read()
         {
            this.entries = new ArrayList<MessageIndexOffset>();
            {
               int i = 0;
               while (!this._io.isEof())
               {
                  this.entries.add(new MessageIndexOffset(this._io, this, _root));
                  i++;
               }
            }
         }

         private ArrayList<MessageIndexOffset> entries;
         private Mcap _root;
         private Mcap.ChunkIndex _parent;

         public ArrayList<MessageIndexOffset> entries()
         {
            return entries;
         }

         public Mcap _root()
         {
            return _root;
         }

         public Mcap.ChunkIndex _parent()
         {
            return _parent;
         }
      }

      private Record chunk;

      public Record chunk()
      {
         if (this.chunk != null)
            return this.chunk;
         KaitaiStream io = _root()._io();
         long _pos = io.pos();
         io.seek(ofsChunk());
         this._raw_chunk = io.readBytes(lenChunk());
         KaitaiStream _io__raw_chunk = new ByteBufferKaitaiStream(_raw_chunk);
         this.chunk = new Record(_io__raw_chunk, this, _root);
         io.seek(_pos);
         return this.chunk;
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
      private Mcap _root;
      private Mcap.Record _parent;
      private byte[] _raw_messageIndexOffsets;
      private byte[] _raw_chunk;

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

      public Mcap _root()
      {
         return _root;
      }

      public Mcap.Record _parent()
      {
         return _parent;
      }

      public byte[] _raw_messageIndexOffsets()
      {
         return _raw_messageIndexOffsets;
      }

      public byte[] _raw_chunk()
      {
         return _raw_chunk;
      }
   }

   private Record footer;

   public Record footer()
   {
      if (this.footer != null)
         return this.footer;
      long _pos = this._io.pos();
      this._io.seek(ofsFooter());
      this._raw_footer = this._io.readBytesFull();
      KaitaiStream _io__raw_footer = new ByteBufferKaitaiStream(_raw_footer);
      this.footer = new Record(_io__raw_footer, this, _root);
      this._io.seek(_pos);
      return this.footer;
   }

   private Integer ofsFooter;

   public Integer ofsFooter()
   {
      if (this.ofsFooter != null)
         return this.ofsFooter;
      int _tmp = (int) (((((_io().size() - 1) - 8) - 20) - 8));
      this.ofsFooter = _tmp;
      return this.ofsFooter;
   }

   private Magic headerMagic;
   private ArrayList<Record> records;
   private Magic footerMagic;
   private Mcap _root;
   private KaitaiStruct _parent;
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

   public Mcap _root()
   {
      return _root;
   }

   public KaitaiStruct _parent()
   {
      return _parent;
   }

   public byte[] _raw_footer()
   {
      return _raw_footer;
   }
}
