package us.ihmc.robotDataLogger.websocket.server;

import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Compression;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Records;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.ByteBuffer;
import java.util.List;

public class WebsocketChannelStarterChunk implements Chunk
{
   public static final int START_TIME = -21;
   public static final int END_TIME = -20;
   public static final Compression COMPRESSION = Compression.NONE;

   private Records records;
   private MCAPBuilder mcapBuilder;

   public WebsocketChannelStarterChunk()
   {
   }

   public static WebsocketChannelStarterChunk create(MCAPBuilder mcapBuilder)
   {
      WebsocketChannelStarterChunk chunk = new WebsocketChannelStarterChunk();
      chunk.records = new Records();
      chunk.mcapBuilder = mcapBuilder;
      return chunk;
   }

   public void addYoVariables(List<YoVariable> yoVariables)
   {
      for (int i = 0; i < yoVariables.size(); i++)
      {
         addYoVariable(yoVariables.get(i));
      }
   }

   public void addYoVariable(YoVariable yoVariable)
   {
      records.add(mcapBuilder.getOrCreateVariableChannelRecord(yoVariable));
   }

   public static WebsocketChannelStarterChunk toWebsocketChannelStarterChunk(Chunk chunk)
   {
      if (chunk instanceof WebsocketChannelStarterChunk)
      {
         return (WebsocketChannelStarterChunk) chunk;
      }
      else if (chunk.messageStartTime() == START_TIME && chunk.messageEndTime() == END_TIME)
      {
         if (chunk.compression() != COMPRESSION)
         {
            throw new IllegalArgumentException("Invalid compression: " + chunk.compression());
         }

         if (!chunk.records().stream().allMatch(record -> record.op() == Opcode.CHANNEL))
         {
            throw new IllegalArgumentException("Invalid records: " + chunk.records());
         }

         WebsocketChannelStarterChunk channelStarterChunk = new WebsocketChannelStarterChunk();
         channelStarterChunk.records = chunk.records();
         return channelStarterChunk;
      }

      return null;
   }

   @Override
   public long messageStartTime()
   {
      return START_TIME;
   }

   @Override
   public long messageEndTime()
   {
      return END_TIME;
   }

   @Override
   public long recordsUncompressedLength()
   {
      return records.getElementLength();
   }

   @Override
   public long uncompressedCRC32()
   {
      return records.getCRC32();
   }

   @Override
   public Compression compression()
   {
      return COMPRESSION;
   }

   @Override
   public long recordsCompressedLength()
   {
      return recordsUncompressedLength();
   }

   @Override
   public ByteBuffer getRecordsCompressedBuffer(boolean directBuffer)
   {
      return getRecordsUncompressedBuffer(directBuffer);
   }

   @Override
   public ByteBuffer getRecordsUncompressedBuffer(boolean directBuffer)
   {
      MCAPByteBufferDataOutput recordsOutput = new MCAPByteBufferDataOutput((int) records.getElementLength(), 2, directBuffer);
      records.forEach(element -> element.write(recordsOutput));
      recordsOutput.close();
      return recordsOutput.getBuffer();
   }

   @Override
   public Records records()
   {
      return records;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Chunk other && Chunk.super.equals(other);
   }
}
