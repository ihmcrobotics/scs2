package us.ihmc.scs2.session.mcap.output;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;

public class MCAPBufferedFileChannelOutput implements MCAPDataOutput
{
   static final int DEFAULT_BUFFER_SIZE = 8192;
   public static final boolean DEFAULT_USE_DIRECT_BUFFER = false;

   private final ByteBuffer writingBuffer;
   private final FileChannel fileChannel;

   public MCAPBufferedFileChannelOutput(FileChannel fileChannel)
   {
      this(fileChannel, DEFAULT_BUFFER_SIZE, DEFAULT_USE_DIRECT_BUFFER);
   }

   public MCAPBufferedFileChannelOutput(FileChannel fileChannel, int writingBufferSize, boolean useDirectBuffer)
   {
      this.fileChannel = fileChannel;
      writingBuffer = useDirectBuffer ? ByteBuffer.allocateDirect(writingBufferSize) : ByteBuffer.allocate(writingBufferSize);
      writingBuffer.order(ByteOrder.LITTLE_ENDIAN);
   }

   @Override
   public long position()
   {
      try
      {
         return fileChannel.position() + writingBuffer.position();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public void putLong(long value)
   {
      if (writingBuffer.remaining() < Long.BYTES)
         flush();
      writingBuffer.putLong(value);
   }

   @Override
   public void putInt(int value)
   {
      if (writingBuffer.remaining() < Integer.BYTES)
         flush();
      writingBuffer.putInt(value);
   }

   @Override
   public void putShort(short value)
   {
      if (writingBuffer.remaining() < Short.BYTES)
         flush();
      writingBuffer.putShort(value);
   }

   @Override
   public void putByte(byte value)
   {
      if (writingBuffer.remaining() < Byte.BYTES)
         flush();
      writingBuffer.put(value);
   }

   @Override
   public void putBytes(byte[] bytes, int offset, int length)
   {
      if (writingBuffer.remaining() >= length)
      {
         writingBuffer.put(bytes, offset, length);
         return;
      }

      try
      {
         flush();
         fileChannel.write(ByteBuffer.wrap(bytes, offset, length));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public void putByteBuffer(ByteBuffer byteBuffer)
   {
      if (writingBuffer.remaining() >= byteBuffer.remaining())
      {
         writingBuffer.put(byteBuffer);
         return;
      }

      try
      {
         flush();
         fileChannel.write(byteBuffer);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void flush()
   {
      writingBuffer.flip();
      try
      {
         fileChannel.write(writingBuffer);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      writingBuffer.clear();
   }

   @Override
   public void close()
   {
      flush();
      try
      {
         fileChannel.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public String toString()
   {
      return "MCAPBufferedFileChannelOutput [fileChannel=" + fileChannel + "]";
   }
}
