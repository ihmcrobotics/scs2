package us.ihmc.robotDataLogger.websocket.mcap;

import io.netty.buffer.ByteBuf;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class WebsocketRecord implements Record
{
   private Opcode opcode;
   private long bodyLength;

   private final byte[] bodyData;

   private final WebsocketMessage message;

   public WebsocketRecord(int bufferSize)
   {
      bodyData = new byte[bufferSize];
      message = new WebsocketMessage(ByteBuffer.wrap(bodyData));
   }

   public void initialize(ByteBuf source)
   {
      opcode = Opcode.byId(source.readUnsignedByte());
      bodyLength = source.readLongLE();
      source.readBytes(bodyData);
   }

   @Override
   public Opcode op()
   {
      return opcode;
   }

   @Override
   public long bodyLength()
   {
      return bodyLength;
   }

   @Override
   public <T> T body()
   {
      if (opcode == Opcode.MESSAGE)
      {
         message.initialize((int) bodyLength);
         return (T) message;
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported opcode: " + opcode);
      }
   }

   @Override
   public void write(MCAPDataOutput dataOutput, boolean writeBody)
   {
      dataOutput.putUnsignedByte(opcode.id());
      dataOutput.putLong(bodyLength);
      dataOutput.putBytes(bodyData, 0, (int) bodyLength);
   }

   @Override
   public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
   {
      if (crc32 == null)
         crc32 = new MCAPCRC32Helper();
      crc32.addUnsignedByte(opcode.id());
      crc32.addLong(bodyLength);
      crc32.addBytes(bodyData, 0, (int) bodyLength);
      return crc32;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Record other && Record.super.equals(other);
   }

   /**
    * Same as {@link MutableMessage} but dedicated for the websocket implementation.
    */
   private class WebsocketMessage implements Message
   {
      private int channelId;
      private long sequence;
      private long logTime;
      private long publishTime;
      private int dataOffset;
      private int dataLength;
      private final ByteBuffer source;
      private final MCAPDataInput dataInput;

      public WebsocketMessage(ByteBuffer source)
      {
         this.source = source;
         source.order(ByteOrder.LITTLE_ENDIAN);
         dataInput = MCAPDataInput.wrap(source);
      }

      public void initialize(int length)
      {
         source.position(0);
         source.limit(length);

         channelId = dataInput.getUnsignedShort();
         sequence = dataInput.getUnsignedInt();
         logTime = dataInput.getLong();
         publishTime = dataInput.getLong();
         dataOffset = source.position();
         dataLength = source.remaining();
      }

      @Override
      public int channelId()
      {
         return channelId;
      }

      @Override
      public long sequence()
      {
         return sequence;
      }

      @Override
      public long logTime()
      {
         return logTime;
      }

      @Override
      public long publishTime()
      {
         return publishTime;
      }

      @Override
      public int dataLength()
      {
         return dataLength;
      }

      @Override
      public byte[] messageData()
      {
         throw new UnsupportedOperationException("Use Message.messageBuffer() instead.");
      }

      @Override
      public ByteBuffer messageBuffer()
      {
         source.position(dataOffset);
         source.limit(dataOffset + dataLength);
         return source;
      }

      @Override
      public void write(MCAPDataOutput dataOutput)
      {
         dataOutput.putUnsignedShort(channelId());
         dataOutput.putUnsignedInt(sequence());
         dataOutput.putLong(logTime());
         dataOutput.putLong(publishTime());
         dataOutput.putBytes(messageData(), 0, dataLength());
      }

      @Override
      public MCAPCRC32Helper updateCRC(MCAPCRC32Helper crc32)
      {
         if (crc32 == null)
            crc32 = new MCAPCRC32Helper();
         crc32.addUnsignedShort(channelId());
         crc32.addUnsignedInt(sequence());
         crc32.addLong(logTime());
         crc32.addLong(publishTime());
         crc32.addBytes(messageData(), 0, dataLength());
         return crc32;
      }

      @Override
      public String toString()
      {
         return toString(0);
      }

      @Override
      public boolean equals(Object object)
      {
         return object instanceof Message other && Message.super.equals(other);
      }
   }
}
