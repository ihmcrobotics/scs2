package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.util.List;

public class ChunkIndexDataInputBacked implements ChunkIndex
{
   private final MCAPDataInput dataInput;
   private final long elementLength;
   private final long messageStartTime;
   private final long messageEndTime;
   private final long chunkOffset;
   private final long chunkLength;
   private final long messageIndexOffsetsOffset;
   private final long messageIndexOffsetsLength;
   private WeakReference<List<MessageIndexOffset>> messageIndexOffsetsRef;
   private final long messageIndexLength;
   private final Compression compression;
   private final long recordsCompressedLength;
   private final long recordsUncompressedLength;

   ChunkIndexDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;
      this.elementLength = elementLength;

      dataInput.position(elementPosition);
      messageStartTime = MCAP.checkPositiveLong(dataInput.getLong(), "messageStartTime");
      messageEndTime = MCAP.checkPositiveLong(dataInput.getLong(), "messageEndTime");
      chunkOffset = MCAP.checkPositiveLong(dataInput.getLong(), "chunkOffset");
      chunkLength = MCAP.checkPositiveLong(dataInput.getLong(), "chunkLength");
      messageIndexOffsetsLength = dataInput.getUnsignedInt();
      messageIndexOffsetsOffset = dataInput.position();
      dataInput.skip(messageIndexOffsetsLength);
      messageIndexLength = MCAP.checkPositiveLong(dataInput.getLong(), "messageIndexLength");
      compression = Compression.fromString(dataInput.getString());
      recordsCompressedLength = MCAP.checkPositiveLong(dataInput.getLong(), "compressedSize");
      recordsUncompressedLength = MCAP.checkPositiveLong(dataInput.getLong(), "uncompressedSize");
   }

   @Override
   public long getElementLength()
   {
      return elementLength;
   }

   private WeakReference<Record> chunkRef;

   @Override
   public Record chunk()
   {
      Record chunk = chunkRef == null ? null : chunkRef.get();

      if (chunk == null)
      {
         chunk = new RecordDataInputBacked(dataInput, chunkOffset);
         chunkRef = new WeakReference<>(chunk);
      }
      return chunkRef.get();
   }

   @Override
   public long messageStartTime()
   {
      return messageStartTime;
   }

   @Override
   public long messageEndTime()
   {
      return messageEndTime;
   }

   @Override
   public long chunkOffset()
   {
      return chunkOffset;
   }

   @Override
   public long chunkLength()
   {
      return chunkLength;
   }

   @Override
   public long messageIndexOffsetsLength()
   {
      return messageIndexOffsetsLength;
   }

   @Override
   public List<MessageIndexOffset> messageIndexOffsets()
   {
      List<MessageIndexOffset> messageIndexOffsets = messageIndexOffsetsRef == null ? null : messageIndexOffsetsRef.get();

      if (messageIndexOffsets == null)
      {
         messageIndexOffsets = MCAP.parseList(dataInput, MessageIndexOffset::new, messageIndexOffsetsOffset, messageIndexOffsetsLength);
         messageIndexOffsetsRef = new WeakReference<>(messageIndexOffsets);
      }

      return messageIndexOffsets;
   }

   @Override
   public long messageIndexLength()
   {
      return messageIndexLength;
   }

   @Override
   public Compression compression()
   {
      return compression;
   }

   @Override
   public long recordsCompressedLength()
   {
      return recordsCompressedLength;
   }

   @Override
   public long recordsUncompressedLength()
   {
      return recordsUncompressedLength;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof ChunkIndex other && ChunkIndex.super.equals(other);
   }
}
