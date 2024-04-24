package us.ihmc.robotDataLogger.websocket.mcap;

import io.netty.buffer.ByteBuf;
import us.ihmc.scs2.session.mcap.encoding.MCAPCRC32Helper;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;

import java.nio.ByteBuffer;

/**
 * Same as {@link MutableMessage} but dedicated for the websocket implementation.
 */
public class WebsocketMessage implements Message
{
   private int channelId;
   private long sequence;
   private long logTime;
   private long publishTime;
   private int dataLength;
   private final byte[] messageData;
   private final ByteBuffer messageDataBuffer;

   public WebsocketMessage(int capacity)
   {
      messageData = new byte[capacity];
      messageDataBuffer = ByteBuffer.wrap(messageData);
   }

   public void initialize(ByteBuf source)
   {
      channelId = source.readUnsignedShortLE();
      sequence = source.readUnsignedIntLE();
      logTime = source.readLongLE();
      publishTime = source.readLongLE();
      dataLength = source.readableBytes();
      messageDataBuffer.clear();
      source.readBytes(messageDataBuffer);
      messageDataBuffer.flip();
   }

   @Override
   public int channelId()
   {
      return channelId;
   }

   public void setChannelId(int channelId)
   {
      this.channelId = channelId;
   }

   @Override
   public long sequence()
   {
      return sequence;
   }

   public void setSequence(long sequence)
   {
      this.sequence = sequence;
   }

   @Override
   public long logTime()
   {
      return logTime;
   }

   public void setLogTime(long logTime)
   {
      this.logTime = logTime;
   }

   @Override
   public long publishTime()
   {
      return publishTime;
   }

   public void setPublishTime(long publishTime)
   {
      this.publishTime = publishTime;
   }

   @Override
   public int dataLength()
   {
      return dataLength;
   }

   public void setDataLength(int dataLength)
   {
      this.dataLength = dataLength;
   }

   @Override
   public byte[] messageData()
   {
      return messageData;
   }

   @Override
   public ByteBuffer messageBuffer()
   {
      return messageDataBuffer;
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
