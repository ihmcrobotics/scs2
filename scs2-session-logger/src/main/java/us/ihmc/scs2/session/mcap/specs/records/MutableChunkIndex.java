package us.ihmc.scs2.session.mcap.specs.records;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class MutableChunkIndex implements ChunkIndex
{
   private Record chunk;
   private long messageStartTime;
   private long messageEndTime;
   private long chunkOffset;
   private long chunkLength;
   private long messageIndexOffsetsLength;
   private List<MessageIndexOffset> messageIndexOffsets;
   private long messageIndexLength;
   private Compression compression;
   private long recordsCompressedLength;
   private long recordsUncompressedLength;

   @Override
   public Record chunk()
   {
      return chunk;
   }

   public void setChunk(Record chunk)
   {
      this.chunk = chunk;
      Chunk body = chunk.body();
      messageStartTime = body.messageStartTime();
      messageEndTime = body.messageEndTime();
      chunkLength = chunk.getElementLength();
      // Resets the message index offsets, they have to be set manually.
      messageIndexOffsets = new ArrayList<>();
      messageIndexOffsetsLength = 0L;
      compression = body.compression();
      recordsCompressedLength = body.recordsCompressedLength();
      recordsUncompressedLength = body.recordsUncompressedLength();
   }

   @Override
   public long messageStartTime()
   {
      return messageStartTime;
   }

   public void setMessageStartTime(long messageStartTime)
   {
      this.messageStartTime = messageStartTime;
   }

   @Override
   public long messageEndTime()
   {
      return messageEndTime;
   }

   public void setMessageEndTime(long messageEndTime)
   {
      this.messageEndTime = messageEndTime;
   }

   @Override
   public long chunkOffset()
   {
      return chunkOffset;
   }

   public void setChunkOffset(long chunkOffset)
   {
      this.chunkOffset = chunkOffset;
   }

   @Override
   public long chunkLength()
   {
      return chunkLength;
   }

   public void setChunkLength(long chunkLength)
   {
      this.chunkLength = chunkLength;
   }

   @Override
   public long messageIndexOffsetsLength()
   {
      return messageIndexOffsetsLength;
   }

   public void setMessageIndexOffsetsLength(long messageIndexOffsetsLength)
   {
      this.messageIndexOffsetsLength = messageIndexOffsetsLength;
   }

   @Override
   public List<MessageIndexOffset> messageIndexOffsets()
   {
      return messageIndexOffsets;
   }

   public void setMessageIndexOffsets(List<MessageIndexOffset> messageIndexOffsets)
   {
      this.messageIndexOffsets = messageIndexOffsets;
      messageIndexOffsetsLength = messageIndexOffsets.stream().mapToLong(MessageIndexOffset::getElementLength).sum();
   }

   @Override
   public long messageIndexLength()
   {
      return messageIndexLength;
   }

   public void setMessageIndexLength(long messageIndexLength)
   {
      this.messageIndexLength = messageIndexLength;
   }

   @Override
   public Compression compression()
   {
      return compression;
   }

   public void setCompression(Compression compression)
   {
      this.compression = compression;
   }

   @Override
   public long recordsCompressedLength()
   {
      return recordsCompressedLength;
   }

   public void setRecordsCompressedLength(long recordsCompressedLength)
   {
      this.recordsCompressedLength = recordsCompressedLength;
   }

   @Override
   public long recordsUncompressedLength()
   {
      return recordsUncompressedLength;
   }

   public void setRecordsUncompressedLength(long recordsUncompressedLength)
   {
      this.recordsUncompressedLength = recordsUncompressedLength;
   }

   @Override
   public long getElementLength()
   {
      Objects.requireNonNull(messageIndexOffsets, "The message index offsets must be set before calling this method.");
      Objects.requireNonNull(compression, "The compression must be set before calling this method.");
      return 7 * Long.BYTES + Integer.BYTES + messageIndexOffsetsLength + compression.getLength();
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
