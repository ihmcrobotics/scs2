package us.ihmc.scs2.session.mcap.encoding;

import net.jpountz.lz4.LZ4Compressor;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.xxhash.XXHash32;
import net.jpountz.xxhash.XXHashFactory;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameDecoder.BD;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameDecoder.BLOCKSIZE;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameDecoder.FLG;
import us.ihmc.scs2.session.mcap.encoding.LZ4FrameDecoder.FrameInfo;

import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

/**
 * This class is a modified version of the original LZ4FrameOutputStream from the lz4-java project.
 * <p>
 * This version allows to decode a byte array into another byte array, without using any {@link OutputStream}.
 * </p>
 */
public class LZ4FrameEncoder
{
   static final FLG.Bits[] DEFAULT_FEATURES = new FLG.Bits[] {FLG.Bits.BLOCK_INDEPENDENCE};

   static final String CLOSED_STREAM = "The stream is already closed";

   private final LZ4Compressor compressor;
   private final XXHash32 checksum;
   private final ByteBuffer blockBuffer; // Buffer for uncompressed input data
   private final byte[] compressedBuffer; // Only allocated once so it can be reused
   private final int maxBlockSize;
   private final long knownSize;
   private final ByteBuffer intLEBuffer = ByteBuffer.allocate(LZ4FrameDecoder.INTEGER_BYTES).order(ByteOrder.LITTLE_ENDIAN);

   private FrameInfo frameInfo = null;

   /**
    * Creates a new encoder that will compress data of unknown size using the LZ4 algorithm.
    *
    * @param blockSize the BLOCKSIZE to use
    * @param bits      a set of features to use
    * @see #LZ4FrameEncoder(BLOCKSIZE, long, FLG.Bits...)
    */
   public LZ4FrameEncoder(BLOCKSIZE blockSize, FLG.Bits... bits)
   {
      this(blockSize, -1L, bits);
   }

   /**
    * Creates a new encoder that will compress data using using fastest instances of {@link LZ4Compressor} and {@link XXHash32}.
    *
    * @param blockSize the BLOCKSIZE to use
    * @param knownSize the size of the uncompressed data. A value less than zero means unknown.
    * @param bits      a set of features to use
    */
   public LZ4FrameEncoder(BLOCKSIZE blockSize, long knownSize, FLG.Bits... bits)
   {
      this(blockSize, knownSize, LZ4Factory.fastestInstance().fastCompressor(), XXHashFactory.fastestInstance().hash32(), bits);
   }

   /**
    * Creates a new encoder that will compress data using the specified instances of {@link LZ4Compressor} and {@link XXHash32}.
    *
    * @param blockSize  the BLOCKSIZE to use
    * @param knownSize  the size of the uncompressed data. A value less than zero means unknown.
    * @param compressor the {@link LZ4Compressor} instance to use to compress data
    * @param checksum   the {@link XXHash32} instance to use to check data for integrity
    * @param bits       a set of features to use
    */
   public LZ4FrameEncoder(BLOCKSIZE blockSize, long knownSize, LZ4Compressor compressor, XXHash32 checksum, FLG.Bits... bits)
   {
      this.compressor = compressor;
      this.checksum = checksum;
      frameInfo = new FrameInfo(new FLG(FLG.DEFAULT_VERSION, bits), new BD(blockSize));
      maxBlockSize = frameInfo.getBD().getBlockMaximumSize();
      blockBuffer = ByteBuffer.allocate(maxBlockSize).order(ByteOrder.LITTLE_ENDIAN);
      compressedBuffer = new byte[this.compressor.maxCompressedLength(maxBlockSize)];
      if (frameInfo.getFLG().isEnabled(FLG.Bits.CONTENT_SIZE) && knownSize < 0)
      {
         throw new IllegalArgumentException("Known size must be greater than zero in order to use the known size feature");
      }
      this.knownSize = knownSize;
   }

   /**
    * Creates a new encoder that will compress data using the LZ4 algorithm. The block independence flag is set, and none of the other flags are
    * set.
    *
    * @param blockSize the BLOCKSIZE to use
    * @see #LZ4FrameEncoder(BLOCKSIZE, FLG.Bits...)
    */
   public LZ4FrameEncoder(BLOCKSIZE blockSize)
   {
      this(blockSize, DEFAULT_FEATURES);
   }

   /**
    * Creates a new encoder that will compress data using the LZ4 algorithm with 4-MB blocks.
    *
    * @see #LZ4FrameEncoder(BLOCKSIZE)
    */
   public LZ4FrameEncoder()
   {
      this(BLOCKSIZE.SIZE_4MB);
   }

   public byte[] encode(byte[] in, byte[] out)
   {
      return encode(in, 0, in.length, out, 0);
   }

   public byte[] encode(byte[] in, int inOffset, int inLength, byte[] out, int outOffset)
   {
      ByteBuffer resultBuffer = encode(ByteBuffer.wrap(in, inOffset, inLength), out == null ? null : ByteBuffer.wrap(out, outOffset, out.length - outOffset));
      if (resultBuffer == null)
         return null;
      byte[] result = new byte[resultBuffer.remaining()];
      resultBuffer.get(result);
      return result;
   }

   public ByteBuffer encode(ByteBuffer in, ByteBuffer out)
   {
      return encode(in, 0, in.remaining(), out, 0);
   }

   public ByteBuffer encode(ByteBuffer in, int inOffset, int inLength, ByteBuffer out, int outOffset)
   {
      int limitPrev = in.limit();
      in.position(inOffset);
      in.limit(inOffset + inLength);
      if (out != null)
      {
         out.order(ByteOrder.LITTLE_ENDIAN);
         out.position(outOffset);
      }

      try
      {
         if (out != null)
         {
            writeHeader(out);
            ensureNotFinished();

            // while b will fill the buffer
            while (in.remaining() > blockBuffer.remaining())
            {
               int sizeWritten = blockBuffer.remaining();
               // fill remaining space in buffer
               blockBuffer.put(in.slice(in.position(), sizeWritten));
               in.position(in.position() + sizeWritten);
               writeBlock(out);
            }
            blockBuffer.put(in);
            writeBlock(out);
            writeEndMark(out);
            out.flip();
            return out;
         }
         else
         {
            ByteBuffer whenOutIsNull = ByteBuffer.allocate(inLength + LZ4FrameDecoder.LZ4_MAX_HEADER_LENGTH).order(ByteOrder.LITTLE_ENDIAN);
            writeHeader(whenOutIsNull);
            ensureNotFinished();

            while (in.remaining() > blockBuffer.remaining())
            {
               int sizeWritten = blockBuffer.remaining();
               // fill remaining space in buffer
               blockBuffer.put(in.slice(in.position(), sizeWritten));
               in.position(in.position() + sizeWritten);

               whenOutIsNull = ensureCapacity(whenOutIsNull, blockBuffer.limit());
               writeBlock(whenOutIsNull);
            }
            blockBuffer.put(in);
            whenOutIsNull = ensureCapacity(whenOutIsNull, blockBuffer.limit());
            writeBlock(whenOutIsNull);
            whenOutIsNull = ensureCapacity(whenOutIsNull, 2 * LZ4FrameDecoder.INTEGER_BYTES);
            writeEndMark(whenOutIsNull);

            whenOutIsNull.flip();
            return whenOutIsNull;
         }
      }
      finally
      {
         in.limit(limitPrev);
      }
   }

   private static ByteBuffer ensureCapacity(ByteBuffer buffer, int remainingNeeded)
   {
      if (buffer.remaining() >= remainingNeeded)
         return buffer;

      ByteBuffer extended = ByteBuffer.allocate(buffer.capacity() + remainingNeeded);
      extended.order(ByteOrder.LITTLE_ENDIAN);
      buffer.flip();
      extended.put(buffer);
      return extended;
   }

   /**
    * Writes the magic number and frame descriptor to the underlying {@link OutputStream}.
    */
   private void writeHeader(ByteBuffer out)
   {
      if (out.remaining() < LZ4FrameDecoder.LZ4_MAX_HEADER_LENGTH)
      {
         throw new IllegalArgumentException("The provided buffer is too small to write the header");
      }
      out.order(ByteOrder.LITTLE_ENDIAN);
      out.putInt(LZ4FrameDecoder.MAGIC);
      out.put(frameInfo.getFLG().toByte());
      out.put(frameInfo.getBD().toByte());
      if (frameInfo.isEnabled(FLG.Bits.CONTENT_SIZE))
      {
         out.putLong(knownSize);
      }
      // compute checksum on all descriptor fields
      final int hash = (checksum.hash(out.array(), LZ4FrameDecoder.INTEGER_BYTES, out.position() - LZ4FrameDecoder.INTEGER_BYTES, 0) >> 8) & 0xFF;
      out.put((byte) hash);
   }

   /**
    * Compresses buffered data, optionally computes an XXHash32 checksum, and writes the result to the buffer.
    */
   private void writeBlock(ByteBuffer out)
   {
      if (blockBuffer.position() == 0)
      {
         return;
      }
      // Make sure there's no stale data
      Arrays.fill(compressedBuffer, (byte) 0);

      if (frameInfo.isEnabled(FLG.Bits.CONTENT_CHECKSUM))
      {
         frameInfo.updateStreamHash(blockBuffer.array(), 0, blockBuffer.position());
      }

      int compressedLength = compressor.compress(blockBuffer.array(), 0, blockBuffer.position(), compressedBuffer, 0);
      final byte[] bufferToWrite;
      final int compressMethod;

      // Store block uncompressed if compressed length is greater (incompressible)
      if (compressedLength >= blockBuffer.position())
      {
         compressedLength = blockBuffer.position();
         bufferToWrite = Arrays.copyOf(blockBuffer.array(), compressedLength);
         compressMethod = LZ4FrameDecoder.LZ4_FRAME_INCOMPRESSIBLE_MASK;
      }
      else
      {
         bufferToWrite = compressedBuffer;
         compressMethod = 0;
      }

      // Write content
      out.putInt(compressedLength | compressMethod);
      out.put(bufferToWrite, 0, compressedLength); // TODO bufferToWrite is a copy, we could avoid it

      // Calculate and write block checksum
      if (frameInfo.isEnabled(FLG.Bits.BLOCK_CHECKSUM))
      {
         out.putInt(checksum.hash(bufferToWrite, 0, compressedLength, 0));
      }
      blockBuffer.rewind();
   }

   /**
    * Similar to the {@link #writeBlock(ByteBuffer)} method. Writes a 0-length block (without block checksum) to signal the end
    * of the block stream.
    */
   private void writeEndMark(ByteBuffer out)
   {
      out.order(ByteOrder.LITTLE_ENDIAN);
      out.putInt(0);
      if (frameInfo.isEnabled(FLG.Bits.CONTENT_CHECKSUM))
      {
         out.putInt(0, frameInfo.currentStreamHash());
      }
      frameInfo.finish();
   }

   /**
    * A simple state check to ensure the stream is still open.
    */
   private void ensureNotFinished()
   {
      if (frameInfo.isFinished())
      {
         throw new IllegalStateException(CLOSED_STREAM);
      }
   }
}