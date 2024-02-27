package us.ihmc.scs2.session.mcap.specs.records;

import java.nio.ByteBuffer;

public class MutableMessage implements Message
{
   private int channelId;
   private long sequence;
   private long logTime;
   private long publishTime;
   private long dataOffset;
   private int dataLength;
   private byte[] messageData;

   public MutableMessage(int channelId, byte[] data)
   {
      this.channelId = channelId;
      this.messageData = data;
      this.dataLength = data.length;
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
   public long dataOffset()
   {
      return dataOffset;
   }

   public void setDataOffset(long dataOffset)
   {
      this.dataOffset = dataOffset;
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
      return ByteBuffer.wrap(messageData);
   }

   public void setMessageData(byte[] messageData)
   {
      this.messageData = messageData;
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
