package us.ihmc.robotDataLogger.websocket.server;

import us.ihmc.scs2.session.mcap.output.MCAPByteBufferDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Compression;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Records;

import java.nio.ByteBuffer;

public class WebsocketSchemaStarterChunk implements Chunk
{
   public static final int START_TIME = -11;
   public static final int END_TIME = -10;
   public static final Compression COMPRESSION = Compression.NONE;

   private Records records;

   public WebsocketSchemaStarterChunk()
   {
   }

   public static WebsocketSchemaStarterChunk create(MCAPBuilder mcapBuilder)
   {
      WebsocketSchemaStarterChunk chunk = new WebsocketSchemaStarterChunk();
      chunk.records = new Records(mcapBuilder.getAllSchemas());
      return chunk;
   }

   public static WebsocketSchemaStarterChunk toWebsocketSchemaStarterChunk(Chunk chunk)
   {
      if (chunk instanceof WebsocketSchemaStarterChunk)
      {
         return (WebsocketSchemaStarterChunk) chunk;
      }
      else if (chunk.messageStartTime() == START_TIME && chunk.messageEndTime() == END_TIME)
      {
         if (chunk.compression() != COMPRESSION)
         {
            throw new IllegalArgumentException("Invalid compression: " + chunk.compression());
         }

         if (!chunk.records().stream().allMatch(record -> record.op() == Opcode.SCHEMA))
         {
            throw new IllegalArgumentException("Invalid records: " + chunk.records());
         }

         WebsocketSchemaStarterChunk schemaStarterChunk = new WebsocketSchemaStarterChunk();
         schemaStarterChunk.records = chunk.records();
         return schemaStarterChunk;
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