package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;

class MessageDataInputBacked implements Message
{
   private final MCAPDataInput dataInput;
   private final int channelId;
   private final long sequence;
   private final long logTime;
   private final long publishTime;
   private final long dataOffset;
   private final int dataLength;
   private WeakReference<ByteBuffer> messageBufferRef;
   private WeakReference<byte[]> messageDataRef;

   MessageDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      channelId = dataInput.getUnsignedShort();
      sequence = dataInput.getUnsignedInt();
      logTime = MCAP.checkPositiveLong(dataInput.getLong(), "logTime");
      publishTime = MCAP.checkPositiveLong(dataInput.getLong(), "publishTime");
      dataOffset = dataInput.position();
      dataLength = (int) (elementLength - (Short.BYTES + Integer.BYTES + 2 * Long.BYTES));
      MCAP.checkLength(elementLength, getElementLength());
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

   /**
    * Returns the length of the data portion of this message.
    *
    * @return the length of the data portion of this message.
    */
   @Override
   public int dataLength()
   {
      return dataLength;
   }

   /**
    * Returns the buffer containing this message, the data AND the header.
    *
    * @return the buffer containing this message.
    */
   @Override
   public ByteBuffer messageBuffer()
   {
      ByteBuffer messageBuffer = messageBufferRef == null ? null : messageBufferRef.get();
      if (messageBuffer == null)
      {
         messageBuffer = dataInput.getByteBuffer(dataOffset, dataLength, false);
         messageBufferRef = new WeakReference<>(messageBuffer);
      }
      return messageBuffer;
   }

   @Override
   public byte[] messageData()
   {
      byte[] messageData = messageDataRef == null ? null : messageDataRef.get();

      if (messageData == null)
      {
         messageData = dataInput.getBytes(dataOffset, dataLength);
         messageDataRef = new WeakReference<>(messageData);
      }
      return messageData;
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
