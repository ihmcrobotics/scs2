package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;

public class RecordDataInputBacked implements Record
{

   private final MCAPDataInput dataInput;

   private final Opcode op;
   private final long bodyLength;
   private final long bodyOffset;
   private WeakReference<Object> bodyRef;

   public RecordDataInputBacked(MCAPDataInput dataInput, long elementPosition)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      op = Opcode.byId(dataInput.getUnsignedByte());
      bodyLength = MCAP.checkPositiveLong(dataInput.getLong(), "bodyLength");
      bodyOffset = dataInput.position();
      MCAP.checkLength(getElementLength(), bodyLength + RECORD_HEADER_LENGTH);
   }

   @Override
   public void write(MCAPDataOutput dataOutput, boolean writeBody)
   {
      dataOutput.putUnsignedByte(op.id());
      dataOutput.putLong(bodyLength);
      if (writeBody)
         dataOutput.putBytes(dataInput.getBytes(bodyOffset, (int) bodyLength));
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addUnsignedByte(op.id());
      crc32.addLong(bodyLength);
      crc32.addBytes(dataInput.getBytes(bodyOffset, (int) bodyLength));
      return crc32;
   }

   @Override
   public Opcode op()
   {
      return op;
   }

   @Override
   public long bodyLength()
   {
      return bodyLength;
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> T body()
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
               case MESSAGE -> Message.load(dataInput, bodyOffset, bodyLength);
               case METADATA_INDEX -> new MetadataIndexDataInputBacked(dataInput, bodyOffset, bodyLength);
               case CHUNK -> Chunk.load(dataInput, bodyOffset, bodyLength);
               case SCHEMA -> Schema.load(dataInput, bodyOffset, bodyLength);
               case CHUNK_INDEX -> ChunkIndex.load(dataInput, bodyOffset, bodyLength);
               case DATA_END -> new DataEnd(dataInput, bodyOffset, bodyLength);
               case ATTACHMENT_INDEX -> AttachmentIndex.load(dataInput, bodyOffset, bodyLength);
               case STATISTICS -> Statistics.load(dataInput, bodyOffset, bodyLength);
               case MESSAGE_INDEX -> new MessageIndexDataInputBacked(dataInput, bodyOffset, bodyLength);
               case CHANNEL -> Channel.load(dataInput, bodyOffset, bodyLength);
               case METADATA -> new MetadataDataInputBacked(dataInput, bodyOffset, bodyLength);
               case ATTACHMENT -> Attachment.load(dataInput, bodyOffset, bodyLength);
               case HEADER -> new Header(dataInput, bodyOffset, bodyLength);
               case FOOTER -> new Footer(dataInput, bodyOffset, bodyLength);
               case SUMMARY_OFFSET -> new SummaryOffset(dataInput, bodyOffset, bodyLength);
            };
         }

         bodyRef = new WeakReference<>(body);
      }
      return (T) body;
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
      out += "\n\t-op = " + op();
      out += "\n\t-bodyLength = " + bodyLength;
      out += "\n\t-bodyOffset = " + bodyOffset;
      Object body = body();
      out += "\n\t-body = " + (body == null ? "null" : "\n" + ((MCAPElement) body).toString(indent + 2));
      return MCAPElement.indent(out, indent);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Record other && Record.super.equals(other);
   }
}
