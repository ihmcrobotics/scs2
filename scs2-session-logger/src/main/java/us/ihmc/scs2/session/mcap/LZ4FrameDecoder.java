package us.ihmc.scs2.session.mcap;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.BitSet;
import java.util.Locale;

import net.jpountz.lz4.LZ4Exception;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4SafeDecompressor;
import net.jpountz.xxhash.StreamingXXHash32;

/*
 * Copyright 2020 The Apache Software Foundation and the lz4-java contributors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import net.jpountz.xxhash.XXHash32;
import net.jpountz.xxhash.XXHashFactory;

/**
 * Implementation of the v1.5.1 LZ4 Frame format. This class is NOT thread safe.
 * <p>
 * Not Supported:
 * <ul>
 * <li>Dependent blocks</li>
 * <li>Legacy streams</li>
 * </ul>
 * <p>
 * Originally based on kafka's KafkaLZ4BlockInputStream.
 *
 * @see <a href="https://github.com/lz4/lz4/blob/dev/doc/lz4_Frame_format.md">LZ4 Framing Format
 *       Spec 1.5.1</a>
 */
public class LZ4FrameDecoder
{

   static final String PREMATURE_EOS = "Stream ended prematurely";
   static final String NOT_SUPPORTED = "Stream unsupported";
   static final String BLOCK_HASH_MISMATCH = "Block checksum mismatch";
   static final String DESCRIPTOR_HASH_MISMATCH = "Stream frame descriptor corrupted";
   static final int MAGIC_SKIPPABLE_BASE = 0x184D2A50;

   static final int MAGIC = 0x184D2204;
   static final int LZ4_MAX_HEADER_LENGTH = 4 + // magic
                                            1 + // FLG
                                            1 + // BD
                                            8 + // Content Size
                                            1; // HC
   static final int INTEGER_BYTES = Integer.SIZE >>> 3; // or Integer.BYTES in Java 1.8
   static final int LONG_BYTES = Long.SIZE >>> 3; // or Long.BYTES in Java 1.8
   static final int LZ4_FRAME_INCOMPRESSIBLE_MASK = 0x80000000;

   private final LZ4SafeDecompressor decompressor;
   private final XXHash32 checksum;
   private final byte[] headerArray = new byte[LZ4_MAX_HEADER_LENGTH];
   private final ByteBuffer headerBuffer = ByteBuffer.wrap(headerArray).order(ByteOrder.LITTLE_ENDIAN);
   private byte[] compressedBuffer;
   private byte[] rawBuffer = null;
   private int maxBlockSize = -1;
   private long expectedContentSize = -1L;
   private long totalContentSize = 0L;
   private boolean firstFrameHeaderRead = false;

   private FrameInfo frameInfo = null;

   /**
    * Creates a new {@link InputStream} that will decompress data using fastest instances of
    * {@link LZ4SafeDecompressor} and {@link XXHash32}. This instance will decompress all concatenated
    * frames in their sequential order.
    *
    * @throws IOException if an I/O error occurs
    * @see LZ4Factory#fastestInstance()
    * @see XXHashFactory#fastestInstance()
    */
   public LZ4FrameDecoder()
   {
      this(LZ4Factory.fastestInstance().safeDecompressor(), XXHashFactory.fastestInstance().hash32());
   }

   /**
    * Creates a new {@link InputStream} that will decompress data using the LZ4 algorithm.
    *
    * @param decompressor the decompressor to use
    * @param checksum     the hash function to use
    * @throws IOException if an I/O error occurs
    */
   public LZ4FrameDecoder(LZ4SafeDecompressor decompressor, XXHash32 checksum)
   {
      this.decompressor = decompressor;
      this.checksum = checksum;
   }

   /**
    * Try and load in the next valid frame info. This will skip over skippable frames.
    *
    * @return True if a frame was loaded. False if there are no more frames in the stream.
    */
   private boolean nextFrameInfo(ByteBuffer in)
   {
      while (true)
      {
         if (in.remaining() < INTEGER_BYTES)
            throw new IllegalStateException(PREMATURE_EOS);

         int magic = in.getInt();

         if (magic == MAGIC)
         {
            readHeader(in);
            return true;
         }
         else if ((magic >>> 4) == (MAGIC_SKIPPABLE_BASE >>> 4))
         {
            skippableFrame(in);
         }
         else
         {
            throw new IllegalStateException(NOT_SUPPORTED);
         }
      }
   }

   private void skippableFrame(ByteBuffer in)
   {
      int skipSize = in.getInt();
      in.position(in.position() + skipSize);
      firstFrameHeaderRead = true;
   }

   /**
    * Reads the frame descriptor from the underlying {@link InputStream}.
    *
    * @param in
    */
   private void readHeader(ByteBuffer in)
   {
      headerBuffer.rewind();

      byte flgByte = in.get();
      byte bdByte = in.get();

      FLG flg = FLG.fromByte(flgByte);
      headerBuffer.put(flgByte);
      BD bd = BD.fromByte(bdByte);
      headerBuffer.put(bdByte);

      this.frameInfo = new FrameInfo(flg, bd);

      if (flg.isEnabled(FLG.Bits.CONTENT_SIZE))
      {
         expectedContentSize = in.getLong();
         headerBuffer.putLong(expectedContentSize);
      }
      totalContentSize = 0L;

      // check stream descriptor hash
      byte hash = (byte) ((checksum.hash(headerArray, 0, headerBuffer.position(), 0) >> 8) & 0xFF);
      byte expectedHash = in.get();

      if (hash != expectedHash)
         throw new IllegalStateException(DESCRIPTOR_HASH_MISMATCH);

      maxBlockSize = frameInfo.getBD().getBlockMaximumSize();
      compressedBuffer = new byte[maxBlockSize]; // Reused during different compressions
      rawBuffer = new byte[maxBlockSize];
      firstFrameHeaderRead = true;
   }

   /**
    * Decompress (if necessary) buffered data, optionally computes and validates a XXHash32 checksum,
    * and writes the result to a buffer.
    */
   private ByteBuffer readBlock(ByteBuffer in, ByteBuffer out)
   {
      int blockSize = in.getInt();
      final boolean compressed = (blockSize & LZ4_FRAME_INCOMPRESSIBLE_MASK) == 0;
      blockSize &= ~LZ4_FRAME_INCOMPRESSIBLE_MASK;

      // Check for EndMark
      if (blockSize == 0)
      {
         if (frameInfo.isEnabled(FLG.Bits.CONTENT_CHECKSUM))
         {
            final int contentChecksum = in.getInt();
            if (contentChecksum != frameInfo.currentStreamHash())
               throw new IllegalStateException("Content checksum mismatch");
         }
         if (frameInfo.isEnabled(FLG.Bits.CONTENT_SIZE) && expectedContentSize != totalContentSize)
            throw new IllegalStateException("Size check mismatch");
         frameInfo.finish();
         return null;
      }

      final byte[] tmpBuffer; // Use a temporary buffer, potentially one used for compression
      if (compressed)
      {
         tmpBuffer = compressedBuffer;
      }
      else
      {
         tmpBuffer = rawBuffer;
      }
      if (blockSize > maxBlockSize)
      {
         throw new IllegalStateException(String.format(Locale.ROOT, "Block size %s exceeded max: %s", blockSize, maxBlockSize));
      }

      in.get(tmpBuffer, 0, blockSize);

      // verify block checksum
      if (frameInfo.isEnabled(FLG.Bits.BLOCK_CHECKSUM))
      {
         final int hashCheck = in.getInt();
         if (hashCheck != checksum.hash(tmpBuffer, 0, blockSize, 0))
            throw new IllegalStateException(BLOCK_HASH_MISMATCH);
      }

      final int currentBufferSize;

      if (compressed)
      {
         try
         {
            currentBufferSize = decompressor.decompress(tmpBuffer, 0, blockSize, rawBuffer, 0, rawBuffer.length);
         }
         catch (LZ4Exception e)
         {
            throw new IllegalStateException(e);
         }
      }
      else
      {
         currentBufferSize = blockSize;
      }
      if (frameInfo.isEnabled(FLG.Bits.CONTENT_CHECKSUM))
         frameInfo.updateStreamHash(rawBuffer, 0, currentBufferSize);

      totalContentSize += currentBufferSize;
      if (out != null)
      { // Could check if capacity is big enough, but might just crash to avoid sneaky surprise of a buffer swap.
         out.put(rawBuffer, 0, currentBufferSize);
         return out;
      }
      else
      {
         ByteBuffer blockOut = ByteBuffer.wrap(rawBuffer);
         blockOut.limit(currentBufferSize);
         blockOut.position(0);
         return blockOut;
      }
   }

   public byte[] decode(byte[] in, byte[] out)
   {
      return decode(in, 0, in.length, out, 0);
   }

   public byte[] decode(byte[] in, int inOffset, int inLength, byte[] out, int outOffset)
   {
      ByteBuffer result = decode(ByteBuffer.wrap(in, inOffset, inLength), out == null ? null : ByteBuffer.wrap(out, outOffset, out.length - outOffset));
      return result == null ? null : result.array();
   }

   public ByteBuffer decode(ByteBuffer in, ByteBuffer out)
   {
      return decode(in, 0, in.remaining(), out, 0);
   }

   public ByteBuffer decode(ByteBuffer in, int inOffset, int inLength, ByteBuffer out, int outOffset)
   {
      int limitPrev = in.limit();
      in.position(inOffset);
      in.limit(inOffset + inLength);
      in.order(ByteOrder.LITTLE_ENDIAN);
      if (out != null)
         out.position(outOffset);

      ByteBuffer whenOutIsNull = null;

      try
      {
         while (in.hasRemaining())
         {
            if (!firstFrameHeaderRead || frameInfo.isFinished())
            {
               if (!nextFrameInfo(in))
                  throw new IllegalStateException("Could not find the Frame Descriptor!");
            }
            ByteBuffer blockOut = readBlock(in, out);

            if (blockOut == null)
               break; // Reached the end

            if (out == null)
            {
               if (whenOutIsNull == null)
               {
                  // Need to make a copy of the data as it will be reused for the next blocks.
                  whenOutIsNull = ByteBuffer.allocate(blockOut.remaining());
                  // whenOutIsNull.put(blockOut); <= apparently this does not perform a deep copy.
                  whenOutIsNull.put(0, blockOut, 0, blockOut.limit());
               }
               else
               {
                  ByteBuffer extended = ByteBuffer.allocate(whenOutIsNull.remaining() + blockOut.remaining());
                  extended.put(whenOutIsNull);
                  extended.put(blockOut);
                  whenOutIsNull = extended;
               }
            }
         }
         if (out != null)
         {
            out.flip();
            return out;
         }
         else
         {
            whenOutIsNull.flip();
            return whenOutIsNull;
         }
      }
      finally
      {
         in.limit(limitPrev);
      }
   }

   static class FrameInfo
   {
      private final FLG flg;
      private final BD bd;
      private final StreamingXXHash32 streamHash;
      private boolean finished = false;

      public FrameInfo(FLG flg, BD bd)
      {
         this.flg = flg;
         this.bd = bd;
         this.streamHash = flg.isEnabled(FLG.Bits.CONTENT_CHECKSUM) ? XXHashFactory.fastestInstance().newStreamingHash32(0) : null;
      }

      public boolean isEnabled(FLG.Bits bit)
      {
         return flg.isEnabled(bit);
      }

      public FLG getFLG()
      {
         return this.flg;
      }

      public BD getBD()
      {
         return this.bd;
      }

      public void updateStreamHash(byte[] buff, int off, int len)
      {
         this.streamHash.update(buff, off, len);
      }

      public int currentStreamHash()
      {
         return this.streamHash.getValue();
      }

      public void finish()
      {
         this.finished = true;
      }

      public boolean isFinished()
      {
         return this.finished;
      }
   }

   public static class FLG
   {
      private static final int DEFAULT_VERSION = 1;

      private final BitSet bitSet;
      private final int version;

      public enum Bits
      {
         RESERVED_0(0), RESERVED_1(1), CONTENT_CHECKSUM(2), CONTENT_SIZE(3), BLOCK_CHECKSUM(4), BLOCK_INDEPENDENCE(5);

         private final int position;

         Bits(int position)
         {
            this.position = position;
         }
      }

      public FLG(int version, Bits... bits)
      {
         this.bitSet = new BitSet(8);
         this.version = version;
         if (bits != null)
         {
            for (Bits bit : bits)
            {
               bitSet.set(bit.position);
            }
         }
         validate();
      }

      private FLG(int version, byte b)
      {
         this.bitSet = BitSet.valueOf(new byte[] {b});
         this.version = version;
         validate();
      }

      public static FLG fromByte(byte flg)
      {
         final byte versionMask = (byte) (flg & (3 << 6));
         return new FLG(versionMask >>> 6, (byte) (flg ^ versionMask));
      }

      public byte toByte()
      {
         return (byte) (bitSet.toByteArray()[0] | ((version & 3) << 6));
      }

      private void validate()
      {
         if (bitSet.get(Bits.RESERVED_0.position))
         {
            throw new RuntimeException("Reserved0 field must be 0");
         }
         if (bitSet.get(Bits.RESERVED_1.position))
         {
            throw new RuntimeException("Reserved1 field must be 0");
         }
         if (!bitSet.get(Bits.BLOCK_INDEPENDENCE.position))
         {
            throw new RuntimeException("Dependent block stream is unsupported (BLOCK_INDEPENDENCE must be set)");
         }
         if (version != DEFAULT_VERSION)
         {
            throw new RuntimeException(String.format(Locale.ROOT, "Version %d is unsupported", version));
         }
      }

      public boolean isEnabled(Bits bit)
      {
         return bitSet.get(bit.position);
      }

      public int getVersion()
      {
         return version;
      }
   }

   public static class BD
   {
      private static final int RESERVED_MASK = 0x8F;

      private final BLOCKSIZE blockSizeValue;

      private BD(BLOCKSIZE blockSizeValue)
      {
         this.blockSizeValue = blockSizeValue;
      }

      public static BD fromByte(byte bd)
      {
         int blockMaximumSize = (bd >>> 4) & 7;
         if ((bd & RESERVED_MASK) > 0)
         {
            throw new RuntimeException("Reserved fields must be 0");
         }

         return new BD(BLOCKSIZE.valueOf(blockMaximumSize));
      }

      // 2^(2n+8)
      public int getBlockMaximumSize()
      {
         return 1 << ((2 * blockSizeValue.getIndicator()) + 8);
      }

      public byte toByte()
      {
         return (byte) ((blockSizeValue.getIndicator() & 7) << 4);
      }
   }

   public static enum BLOCKSIZE
   {
      SIZE_64KB(4), SIZE_256KB(5), SIZE_1MB(6), SIZE_4MB(7);

      private final int indicator;

      BLOCKSIZE(int indicator)
      {
         this.indicator = indicator;
      }

      public int getIndicator()
      {
         return this.indicator;
      }

      public static BLOCKSIZE valueOf(int indicator)
      {
         switch (indicator)
         {
            case 7:
               return SIZE_4MB;
            case 6:
               return SIZE_1MB;
            case 5:
               return SIZE_256KB;
            case 4:
               return SIZE_64KB;
            default:
               throw new IllegalArgumentException(String.format(Locale.ROOT, "Block size must be 4-7. Cannot use value of [%d]", indicator));
         }
      }
   }
}
