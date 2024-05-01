package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;

import java.util.List;
import java.util.Objects;

/**
 * MCAP files may contain a variety of records.
 * Records are identified by a single-byte opcode.
 * Record opcodes in the range 0x01-0x7F are reserved for future MCAP format usage.
 * 0x80-0xFF are reserved for application extensions and user proposals.
 * 0x00 is not a valid opcode.
 *
 * @see <a href="https://mcap.dev/spec#records">MCAP Records</a>
 */
public interface Record extends MCAPElement
{
   int RECORD_HEADER_LENGTH = 9;

   static Record load(MCAPDataInput dataInput)
   {
      return load(dataInput, dataInput.position());
   }

   static Record load(MCAPDataInput dataInput, long elementPosition)
   {
      return new RecordDataInputBacked(dataInput, elementPosition);
   }

   Opcode op();

   long bodyLength();

   <T> T body();

   @Override
   default long getElementLength()
   {
      return RECORD_HEADER_LENGTH + bodyLength();
   }

   @Override
   default String toString(int indent)
   {
      String out = getClass().getSimpleName() + ":";
      out += "\n\t-op = " + op();
      Object body = body();
      out += "\n\t-body = " + (body == null ? "null" : "\n" + ((MCAPElement) body).toString(indent + 2));
      return MCAPElement.indent(out, indent);
   }

   @Override
   default void write(MCAPDataOutput dataOutput)
   {
      write(dataOutput, true);
   }

   void write(MCAPDataOutput dataOutput, boolean writeBody);

   default Record generateMetadataIndexRecord(long metadataOffset)
   {
      if (op() != Opcode.METADATA)
         throw new UnsupportedOperationException("Cannot generate a metadata index record from a non-metadata record");

      MutableMetadataIndex metadataIndex = new MutableMetadataIndex();
      metadataIndex.setMetadataOffset(metadataOffset);
      metadataIndex.setMetadata(this);
      return new MutableRecord(metadataIndex);
   }

   default Record generateAttachmentIndexRecord(long attachmentOffset)
   {
      if (op() != Opcode.ATTACHMENT)
         throw new UnsupportedOperationException("Cannot generate an attachment index record from a non-attachment record");

      MutableAttachmentIndex attachmentIndex = new MutableAttachmentIndex();
      attachmentIndex.setAttachmentOffset(attachmentOffset);
      attachmentIndex.setAttachment(this);
      return new MutableRecord(attachmentIndex);
   }

   default Record generateChunkIndexRecord(long chunkOffset, List<? extends Record> messageIndexRecordList)
   {
      if (op() != Opcode.CHUNK)
         throw new UnsupportedOperationException("Cannot generate a chunk index record from a non-chunk record");

      MutableChunkIndex chunkIndex = new MutableChunkIndex();
      chunkIndex.setChunkOffset(chunkOffset);
      chunkIndex.setChunk(this);
      chunkIndex.setMessageIndexOffsets(Records.generateMessageIndexOffsets(chunkOffset + getElementLength(), messageIndexRecordList));
      chunkIndex.setMessageIndexLength(messageIndexRecordList.stream().mapToLong(Record::getElementLength).sum());
      return new MutableRecord(chunkIndex);
   }

   @Override
   default boolean equals(MCAPElement mcapElement)
   {
      if (mcapElement == this)
         return true;

      if (mcapElement instanceof Record other)
      {
         if (op() != other.op())
            return false;
         return Objects.equals(body(), other.body());
      }

      return false;
   }
}
