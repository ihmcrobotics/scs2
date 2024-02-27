package us.ihmc.scs2.session.mcap.input;

import com.github.luben.zstd.ZstdDecompressCtx;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameDecoder;
import us.ihmc.scs2.session.mcap.specs.records.Compression;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;

public class MCAPBufferedFileChannelInput implements MCAPDataInput
{
   static final int DEFAULT_BUFFER_SIZE = 8192;
   // TODO Somehow I would have expected direct buffer to be faster, it wasn't the case on a 10GB file on my office desktop.
   public static final boolean DEFAULT_USE_DIRECT_BUFFER = false;

   private long _pos;
   private final ByteBuffer readingBuffer;
   private final FileChannel fileChannel;

   public MCAPBufferedFileChannelInput(FileChannel fileChannel)
   {
      this(fileChannel, DEFAULT_BUFFER_SIZE, DEFAULT_USE_DIRECT_BUFFER);
   }

   public MCAPBufferedFileChannelInput(FileChannel fileChannel, int readingBufferSize, boolean useDirectBuffer)
   {
      this.fileChannel = fileChannel;
      readingBuffer = useDirectBuffer ? ByteBuffer.allocateDirect(readingBufferSize) : ByteBuffer.allocate(readingBufferSize);
      readingBuffer.order(ByteOrder.LITTLE_ENDIAN);
      try
      {
         _pos = fileChannel.position();
         fileChannel.read(readingBuffer, _pos);
         readingBuffer.flip();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public void position(long newPosition)
   {
      if (newPosition == position())
         return;

      try
      {
         if (newPosition > _pos && newPosition < _pos + readingBuffer.limit())
         {
            readingBuffer.position((int) (newPosition - _pos));
            return;
         }

         _pos = newPosition;
         readingBuffer.clear();
         fileChannel.read(readingBuffer, _pos);
         readingBuffer.flip();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public long position()
   {
      return _pos + readingBuffer.position();
   }

   @Override
   public long size()
   {
      try
      {
         return fileChannel.size();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public long getLong()
   {
      if (readingBuffer.remaining() < Long.BYTES)
         fillBuffer();
      return readingBuffer.getLong();
   }

   @Override
   public int getInt()
   {
      if (readingBuffer.remaining() < Integer.BYTES)
         fillBuffer();
      return readingBuffer.getInt();
   }

   @Override
   public short getShort()
   {
      if (readingBuffer.remaining() < Short.BYTES)
         fillBuffer();
      return readingBuffer.getShort();
   }

   @Override
   public byte getByte()
   {
      if (readingBuffer.remaining() < Byte.BYTES)
         fillBuffer();
      return readingBuffer.get();
   }

   @Override
   public void getBytes(byte[] bytes)
   {
      int length = bytes.length;
      int remaining = length;

      if (size() - position() < length)
         throw new IndexOutOfBoundsException(
               "End of file reached. Requested: " + length + ", remaining: " + (size() - position()) + ", position: " + position() + ", size: " + size());

      while (remaining > 0)
      {
         if (readingBuffer.remaining() < remaining)
         {
            int toRead = readingBuffer.remaining();
            readingBuffer.get(bytes, length - remaining, toRead);
            remaining -= toRead;
            fillBuffer();
         }
         else
         {
            readingBuffer.get(bytes, length - remaining, remaining);
            remaining = 0;
         }
      }
   }

   @Override
   public byte[] getBytes(long offset, int length)
   {
      byte[] bytes = new byte[length];

      try
      {
         if (offset >= _pos && offset + length < _pos + readingBuffer.limit())
            readingBuffer.get((int) (offset - _pos), bytes);
         else
            fileChannel.read(ByteBuffer.wrap(bytes), offset);
         return bytes;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public ByteBuffer getByteBuffer(long offset, int length, boolean direct)
   {
      try
      {
         ByteBuffer buffer = direct ? ByteBuffer.allocateDirect(length) : ByteBuffer.allocate(length);
         buffer.order(ByteOrder.LITTLE_ENDIAN);
         fileChannel.read(buffer, offset);
         buffer.flip();
         return buffer;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public ByteBuffer getDecompressedByteBuffer(long offset, int compressedLength, int uncompressedLength, Compression compression, boolean direct)
   {
      if (compression == Compression.NONE)
         return getByteBuffer(offset, uncompressedLength, direct);

      ByteBuffer compressedBuffer = getByteBuffer(offset,
                                                  compressedLength,
                                                  compression == Compression.ZSTD); // TODO Try to use internal buffer instead when possible
      ByteBuffer decompressedBuffer;

      if (compression == Compression.LZ4)
      {
         LZ4FrameDecoder lz4FrameDecoder = new LZ4FrameDecoder();
         decompressedBuffer = direct ? ByteBuffer.allocateDirect(uncompressedLength) : ByteBuffer.allocate(uncompressedLength);
         lz4FrameDecoder.decode(compressedBuffer, 0, compressedLength, decompressedBuffer, 0);
      }
      else if (compression == Compression.ZSTD)
      {
         try (ZstdDecompressCtx zstdDecompressCtx = new ZstdDecompressCtx())
         {
            decompressedBuffer = zstdDecompressCtx.decompress(compressedBuffer, uncompressedLength);
         }
      }
      else
      {
         throw new IllegalArgumentException("Unsupported compression: " + compression);
      }

      decompressedBuffer.order(ByteOrder.LITTLE_ENDIAN);
      return decompressedBuffer;
   }

   private void fillBuffer()
   {
      try
      {
         // The reading position in the file is gonna be _pos + readingBuffer.limit()
         long fileReadingPosition = _pos + readingBuffer.limit();
         readingBuffer.compact();
         int bytesRead = fileChannel.read(readingBuffer, fileReadingPosition);
         readingBuffer.flip();
         _pos = fileReadingPosition + bytesRead - readingBuffer.limit();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   // For testing purpose
   long _pos()
   {
      return _pos;
   }

   // For testing purpose
   ByteBuffer getReadingBuffer()
   {
      return readingBuffer;
   }

   @Override
   public String toString()
   {
      return "MCAPBufferedFileChannelInput{" + "readingBuffer=" + readingBuffer + ", fileChannel=" + fileChannel + '}';
   }
}
