package us.ihmc.scs2.session.mcap.specs.records;

import gnu.trove.map.hash.TIntObjectHashMap;

public enum Opcode
{
   /**
    * Header is the first record in an MCAP file.
    *
    * @see Header
    */
   HEADER(1),
   /**
    * Footer records contain end-of-file information. MCAP files must end with a Footer record.
    *
    * @see Footer
    */
   FOOTER(2),

   /**
    * A Schema record defines an individual schema.
    * Schema records are uniquely identified within a file by their schema ID.
    * A Schema record must occur at least once in the file prior to any Channel referring to its ID.
    * Any two schema records sharing a common ID must be identical.
    *
    * @see Schema
    */
   SCHEMA(3),
   /**
    * Channel records define encoded streams of messages on topics.
    * Channel records are uniquely identified within a file by their channel ID.
    * A Channel record must occur at least once in the file prior to any message referring to its channel ID.
    * Any two channel records sharing a common ID must be identical.
    *
    * @see Channel
    */
   CHANNEL(4),
   /**
    * Message records encode a single timestamped message on a channel.
    * The message encoding and schema must match that of the Channel record corresponding to the message's channel ID.
    *
    * @see Message
    */
   MESSAGE(5),
   /**
    * Chunk records each contain a batch of Schema, Channel, and Message records.
    * The batch of records contained in a chunk may be compressed or uncompressed.
    * All messages in the chunk must reference channels recorded earlier in the file (in a previous chunk or earlier in the current chunk).
    *
    * @see Chunk
    */
   CHUNK(6),
   /**
    * MessageIndex records allow readers to locate individual records within a chunk by timestamp.
    * A sequence of Message Index records occurs immediately after each chunk.
    * Exactly one Message Index record must exist in the sequence for every channel on which a message occurs inside the chunk.
    *
    * @see MessageIndex
    */
   MESSAGE_INDEX(7),
   /**
    * ChunkIndex records contain the location of a Chunk record and its associated MessageIndex records.
    * A ChunkIndex record exists for every Chunk in the file.
    *
    * @see ChunkIndex
    */
   CHUNK_INDEX(8),
   /**
    * Attachment records contain auxiliary artifacts such as text, core dumps, calibration data, or other arbitrary data.
    * Attachment records must not appear within a chunk.
    *
    * @see Attachment
    */
   ATTACHMENT(9),
   /**
    * An Attachment Index record contains the location of an attachment in the file.
    * An Attachment Index record exists for every Attachment record in the file.
    *
    * @see AttachmentIndex
    */
   ATTACHMENT_INDEX(10),
   /**
    * A Statistics record contains summary information about the recorded data.
    * The statistics record is optional, but the file should contain at most one.
    *
    * @see Statistics
    */
   STATISTICS(11),
   /**
    * A metadata record contains arbitrary user data in key-value pairs.
    *
    * @see MetadataDataInputBacked
    */
   METADATA(12),
   /**
    * A metadata index record contains the location of a metadata record within the file.
    *
    * @see MetadataIndex
    */
   METADATA_INDEX(13),
   /**
    * A Summary Offset record contains the location of records within the summary section.
    * Each Summary Offset record corresponds to a group of summary records with the same opcode.
    *
    * @see SummaryOffset
    */
   SUMMARY_OFFSET(14),
   /**
    * A Data End record indicates the end of the data section.
    *
    * @see DataEnd
    */
   DATA_END(15);

   private final int id;

   Opcode(int id)
   {
      this.id = id;
   }

   public int id()
   {
      return id;
   }

   private static final TIntObjectHashMap<Opcode> byId = new TIntObjectHashMap<>(15);

   static
   {
      for (Opcode e : Opcode.values())
         byId.put(e.id(), e);
   }

   public static Opcode byId(int id)
   {
      return byId.get(id);
   }

   public static Opcode byBodyType(Class<?> bodyType)
   {
      if (bodyType == null)
         return null;
      if (Header.class.isAssignableFrom(bodyType))
         return HEADER;
      if (Footer.class.isAssignableFrom(bodyType))
         return FOOTER;
      if (Schema.class.isAssignableFrom(bodyType))
         return SCHEMA;
      if (Channel.class.isAssignableFrom(bodyType))
         return CHANNEL;
      if (Message.class.isAssignableFrom(bodyType))
         return MESSAGE;
      if (Chunk.class.isAssignableFrom(bodyType))
         return CHUNK;
      if (MessageIndex.class.isAssignableFrom(bodyType))
         return MESSAGE_INDEX;
      if (ChunkIndex.class.isAssignableFrom(bodyType))
         return CHUNK_INDEX;
      if (Attachment.class.isAssignableFrom(bodyType))
         return ATTACHMENT;
      if (AttachmentIndex.class.isAssignableFrom(bodyType))
         return ATTACHMENT_INDEX;
      if (Statistics.class.isAssignableFrom(bodyType))
         return STATISTICS;
      if (Metadata.class.isAssignableFrom(bodyType))
         return METADATA;
      if (MetadataIndex.class.isAssignableFrom(bodyType))
         return METADATA_INDEX;
      if (SummaryOffset.class.isAssignableFrom(bodyType))
         return SUMMARY_OFFSET;
      if (DataEnd.class.isAssignableFrom(bodyType))
         return DATA_END;
      throw new IllegalArgumentException("Unsupported body type: " + bodyType);
   }
}
