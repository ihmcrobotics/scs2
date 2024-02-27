package us.ihmc.scs2.session.mcap.input;

import com.github.luben.zstd.ZstdCompressCtx;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.specs.records.Compression;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.channels.ReadableByteChannel;
import java.nio.channels.WritableByteChannel;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class MCAPBufferedFileChannelInputTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testReadingBytes()
   {
      ByteBuffer buffer = ByteBuffer.wrap("Hello, World!".getBytes());
      MCAPBufferedFileChannelInput input = new MCAPBufferedFileChannelInput(mockFileChannel(buffer));

      byte[] bytes = new byte[5];
      input.getBytes(bytes);

      assertArrayEquals("Hello".getBytes(), bytes);
   }

   @Test
   public void testRandomAccessesOnLargeByteArray()
   {
      Random random = new Random(1234);
      byte[] originalBytes = new byte[100000];
      random.nextBytes(originalBytes);

      ByteBuffer buffer = ByteBuffer.wrap(originalBytes);
      MCAPBufferedFileChannelInput input = new MCAPBufferedFileChannelInput(mockFileChannel(buffer), 1024, false);

      for (int i = 0; i < ITERATIONS; i++)
      {
         { // Test getBytes
            int position = random.nextInt(originalBytes.length);
            int length = random.nextInt(originalBytes.length - position);
            byte[] expectedBytes = new byte[length];
            System.arraycopy(originalBytes, position, expectedBytes, 0, length);

            byte[] actualBytes = new byte[length];
            input.position(position);
            input.getBytes(actualBytes);

            assertArrayEquals(expectedBytes, actualBytes, "Iteration " + i);
         }

         { // Test getByteBuffer
            int position = random.nextInt(originalBytes.length);
            int length = random.nextInt(originalBytes.length - position);
            byte[] expectedBytes = new byte[length];
            System.arraycopy(originalBytes, position, expectedBytes, 0, length);

            ByteBuffer actualBuffer = input.getByteBuffer(position, length, false);
            byte[] actualBytes = new byte[length];
            actualBuffer.get(actualBytes);

            assertArrayEquals(expectedBytes, actualBytes);
         }

         { // Test getLong
            int position = random.nextInt(originalBytes.length - Long.BYTES);
            long expectedValue = ByteBuffer.wrap(originalBytes, position, Long.BYTES).order(ByteOrder.LITTLE_ENDIAN).getLong();

            input.position(position);
            long actualValue = input.getLong();

            assertEquals(expectedValue, actualValue);
         }

         { // Test getInt
            int position = random.nextInt(originalBytes.length - Integer.BYTES);
            int expectedValue = ByteBuffer.wrap(originalBytes, position, Integer.BYTES).order(ByteOrder.LITTLE_ENDIAN).getInt();

            input.position(position);
            int actualValue = input.getInt();

            assertEquals(expectedValue, actualValue);
         }

         { // Test getShort
            int position = random.nextInt(originalBytes.length - Short.BYTES);
            short expectedValue = ByteBuffer.wrap(originalBytes, position, Short.BYTES).order(ByteOrder.LITTLE_ENDIAN).getShort();

            input.position(position);
            short actualValue = input.getShort();

            assertEquals(expectedValue, actualValue);
         }

         { // Test getByte
            int position = random.nextInt(originalBytes.length);
            byte expectedValue = originalBytes[position];

            input.position(position);
            byte actualValue = input.getByte();

            assertEquals(expectedValue, actualValue);
         }
      }
   }

   @Test
   public void testGettingValues() throws IOException
   {// Create a ByteBuffer for testing
      ByteBuffer originalBuffer = ByteBuffer.allocate(24);
      originalBuffer.order(ByteOrder.LITTLE_ENDIAN);
      originalBuffer.putLong(123456789L);
      originalBuffer.putInt(987654321);
      originalBuffer.putLong(9047L);
      originalBuffer.putShort((short) 12345);
      originalBuffer.put((byte) 127);
      originalBuffer.flip();

      { // Test with the reading buffer that is larger than the original buffer
         MCAPBufferedFileChannelInput input = new MCAPBufferedFileChannelInput(mockFileChannel(duplicate(originalBuffer, false)));

         // Assert the values
         assertEquals(123456789L, input.getLong());
         assertEquals(987654321, input.getInt());
         assertEquals(9047L, input.getLong());
         assertEquals((short) 12345, input.getShort());
         assertEquals((byte) 127, input.getByte());
      }

      { // Test with the reading buffer that is smaller than the original buffer
         MCAPBufferedFileChannelInput input = new MCAPBufferedFileChannelInput(mockFileChannel(duplicate(originalBuffer, false)), 8, false);

         // Assert the values
         assertEquals(123456789L, input.getLong());
         assertInputPositions(input, 8, 0, 8);
         assertEquals(987654321, input.getInt());
         assertInputPositions(input, 12, 8, 4);
         assertEquals(9047L, input.getLong());
         assertInputPositions(input, 20, 12, 8);
         assertEquals((short) 12345, input.getShort());
         assertInputPositions(input, 22, 20, 2);
         assertEquals((byte) 127, input.getByte());
      }
   }

   private static void assertInputPositions(MCAPBufferedFileChannelInput input, long expectedPosition, long expected_pos, int expectedReadingBufferPosition)
   {
      assertEquals(expectedPosition, input.position());
      assertEquals(expected_pos, input._pos());
      assertEquals(expectedReadingBufferPosition, input.getReadingBuffer().position());
   }

   @Test
   public void testGettingByteBuffer()
   {
      ByteBuffer buffer = ByteBuffer.wrap("Hello, World!".getBytes());
      MCAPBufferedFileChannelInput input = new MCAPBufferedFileChannelInput(mockFileChannel(buffer));

      // Get byte buffer from the file channel
      ByteBuffer result = input.getByteBuffer(0, buffer.capacity(), false);

      // Assert the byte buffer
      assertArrayEquals(buffer.array(), result.array());
   }

   @Test
   public void testGetDecompressedByteBufferFromFileChannel()
   {
      { // No compression
         ByteBuffer buffer = ByteBuffer.wrap("Hello, World!".getBytes());
         MCAPBufferedFileChannelInput input = new MCAPBufferedFileChannelInput(mockFileChannel(buffer));

         // Get the decompressed byte buffer
         ByteBuffer decompressedBuffer = input.getDecompressedByteBuffer(0, (int) input.size(), (int) input.size(), Compression.NONE, false);

         // Verify the content of the decompressed buffer
         byte[] expectedBytes = "Hello, World!".getBytes();
         byte[] actualBytes = new byte[decompressedBuffer.remaining()];
         decompressedBuffer.get(actualBytes);
         assertArrayEquals(expectedBytes, actualBytes);
      }

      { // ZSTD
         ByteBuffer originalBuffer = duplicate(ByteBuffer.wrap("Hello, World!".getBytes()), true);
         ByteBuffer compressedBuffer = new ZstdCompressCtx().compress(originalBuffer);
         MCAPBufferedFileChannelInput input = new MCAPBufferedFileChannelInput(mockFileChannel(compressedBuffer));

         // Get the decompressed byte buffer
         ByteBuffer decompressedBuffer = input.getDecompressedByteBuffer(0, (int) input.size(), originalBuffer.capacity(), Compression.ZSTD, false);

         // Verify the content of the decompressed buffer
         byte[] expectedBytes = "Hello, World!".getBytes();
         byte[] actualBytes = new byte[decompressedBuffer.remaining()];
         decompressedBuffer.get(actualBytes);
         assertArrayEquals(expectedBytes, actualBytes);
      }
   }

   private static ByteBuffer duplicate(ByteBuffer buffer, boolean direct)
   {
      ByteBuffer duplicate = direct ? ByteBuffer.allocateDirect(buffer.capacity()) : ByteBuffer.allocate(buffer.capacity());
      duplicate.order(buffer.order());
      duplicate.put(buffer.duplicate());
      duplicate.flip();
      return duplicate;
   }

   private static FileChannel mockFileChannel(ByteBuffer content)
   {
      return new FileChannel()
      {
         @Override
         public int read(ByteBuffer dst)
         {
            int remaining = content.remaining();
            int toRead = Math.min(dst.remaining(), remaining);
            for (int i = 0; i < toRead; i++)
            {
               dst.put(content.get());
            }
            return toRead;
         }

         @Override
         public int read(ByteBuffer dst, long position)
         {
            int remaining = (int) (content.limit() - position);
            int toRead = Math.min(dst.remaining(), remaining);
            dst.put(dst.position(), content, (int) position, toRead);
            dst.position(dst.position() + toRead);
            return toRead;
         }

         @Override
         public long position()
         {
            return content.position();
         }

         @Override
         public FileChannel position(long newPosition)
         {
            content.position((int) newPosition);
            return this;
         }

         @Override
         public long size()
         {
            return content.limit();
         }

         @Override
         public int write(ByteBuffer src)
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public int write(ByteBuffer src, long position)
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public MappedByteBuffer map(MapMode mode, long position, long size) throws IOException
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public FileLock lock(long position, long size, boolean shared) throws IOException
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public FileLock tryLock(long position, long size, boolean shared) throws IOException
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public FileChannel truncate(long size)
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public void force(boolean metaData)
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public long transferTo(long position, long count, WritableByteChannel target) throws IOException
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public long transferFrom(ReadableByteChannel src, long position, long count) throws IOException
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public long read(ByteBuffer[] dsts, int offset, int length)
         {
            throw new UnsupportedOperationException();
         }

         @Override
         public long write(ByteBuffer[] srcs, int offset, int length)
         {
            throw new UnsupportedOperationException();
         }

         @Override
         protected void implCloseChannel()
         {
            throw new UnsupportedOperationException();
         }
      };
   }
}
