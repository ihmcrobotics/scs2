/*
 * Copyright 2014 The Netty Project
 *
 * The Netty Project licenses this file to you under the Apache License,
 * version 2.0 (the "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at:
 *
 *   https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
package us.ihmc.scs2.session.mcap;

import static us.ihmc.scs2.session.mcap.Lz4Constants.BLOCK_TYPE_COMPRESSED;
import static us.ihmc.scs2.session.mcap.Lz4Constants.BLOCK_TYPE_NON_COMPRESSED;
import static us.ihmc.scs2.session.mcap.Lz4Constants.COMPRESSION_LEVEL_BASE;
import static us.ihmc.scs2.session.mcap.Lz4Constants.DEFAULT_SEED;
import static us.ihmc.scs2.session.mcap.Lz4Constants.HEADER_LENGTH;
import static us.ihmc.scs2.session.mcap.Lz4Constants.MAGIC_NUMBER;
import static us.ihmc.scs2.session.mcap.Lz4Constants.MAX_BLOCK_SIZE;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.Checksum;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.UnpooledByteBufAllocator;
import io.netty.handler.codec.compression.DecompressionException;
import io.netty.handler.codec.compression.Lz4XXHash32;
import io.netty.util.internal.ObjectUtil;
import net.jpountz.lz4.LZ4Exception;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;

/**
 * https://github.com/lz4/lz4/blob/dev/doc/lz4_Frame_format.md
 */
public class Lz4FrameDecoder
{
   private static final UnpooledByteBufAllocator ALLOCATOR = UnpooledByteBufAllocator.DEFAULT;

   /**
    * Magic number of LZ4 block: "4 Bytes, Little endian format. Value : 0x184D2204"
    */
   private static final int MAGIC_NUMBER = 0x184D2204;

   private static final int MIN_HEADER_LENGTH = 7;

   /**
    * Current state of stream.
    */
   private enum State
   {
      INIT_BLOCK, DECOMPRESS_DATA, FINISHED, CORRUPTED
   }

   private State currentState = State.INIT_BLOCK;

   /**
    * Underlying decompressor in use.
    */
   private LZ4FastDecompressor decompressor;

   /**
    * Underlying checksum calculator in use.
    */
   private ByteBufChecksum checksum;

   /**
    * Type of current block.
    */
   private int blockType;

   /**
    * Compressed length of current incoming block.
    */
   private int compressedLength;

   /**
    * Decompressed length of current incoming block.
    */
   private int decompressedLength;

   /**
    * Checksum value of current incoming block.
    */
   private int currentChecksum;

   /**
    * Creates the fastest LZ4 decoder. Note that by default, validation of the checksum header in each
    * chunk is DISABLED for performance improvements. If performance is less of an issue, or if you
    * would prefer the safety that checksum validation brings, please use the
    * {@link #Lz4FrameDecoder(boolean)} constructor with the argument set to {@code true}.
    */
   public Lz4FrameDecoder()
   {
      this(false);
   }

   /**
    * Creates a LZ4 decoder with fastest decoder instance available on your machine.
    *
    * @param validateChecksums if {@code true}, the checksum field will be validated against the actual
    *                          uncompressed data, and if the checksums do not match, a suitable
    *                          {@link DecompressionException} will be thrown
    */
   public Lz4FrameDecoder(boolean validateChecksums)
   {
      this(LZ4Factory.fastestInstance(), validateChecksums);
   }

   /**
    * Creates a new LZ4 decoder with customizable implementation.
    *
    * @param factory           user customizable {@link LZ4Factory} instance which may be JNI bindings
    *                          to the original C implementation, a pure Java implementation or a Java
    *                          implementation that uses the {@link sun.misc.Unsafe}
    * @param validateChecksums if {@code true}, the checksum field will be validated against the actual
    *                          uncompressed data, and if the checksums do not match, a suitable
    *                          {@link DecompressionException} will be thrown. In this case encoder will
    *                          use xxhash hashing for Java, based on Yann Collet's work available at
    *                          <a href="https://github.com/Cyan4973/xxHash">Github</a>.
    */
   public Lz4FrameDecoder(LZ4Factory factory, boolean validateChecksums)
   {
      this(factory, validateChecksums ? new Lz4XXHash32(DEFAULT_SEED) : null);
   }

   /**
    * Creates a new customizable LZ4 decoder.
    *
    * @param factory  user customizable {@link LZ4Factory} instance which may be JNI bindings to the
    *                 original C implementation, a pure Java implementation or a Java implementation
    *                 that uses the {@link sun.misc.Unsafe}
    * @param checksum the {@link Checksum} instance to use to check data for integrity. You may set
    *                 {@code null} if you do not want to validate checksum of each block
    */
   public Lz4FrameDecoder(LZ4Factory factory, Checksum checksum)
   {
      decompressor = ObjectUtil.checkNotNull(factory, "factory").fastDecompressor();
      this.checksum = checksum == null ? null : ByteBufChecksum.wrapChecksum(checksum);
   }

   public List<ByteBuffer> decode2(ByteBuffer in)
   {
      List<ByteBuffer> out = new ArrayList<>();

      if (in.remaining() < MIN_HEADER_LENGTH)
         return out;

      in.order(ByteOrder.LITTLE_ENDIAN);
      long magic = in.getInt();

      if (magic != MAGIC_NUMBER)
         throw new DecompressionException("unexpected block identifier");

      // Frame Descriptor
      byte flg = in.get();
      boolean dictIDFlag = (flg & 1) == 1;
      // int reserved = (flg >> 1) & 1;
      boolean contentChecksumFlag = ((flg >> 2) & 1) == 1;
      boolean contentSizeFlag = ((flg >> 3) & 1) == 1;
      boolean blockChecksumFlag = ((flg >> 4) & 1) == 1;
      boolean blockIndependenceFlag = ((flg >> 5) & 1) == 1;
      int version = flg >> 6;

      byte bd = in.get();

      return out;
   }

   public List<ByteBuf> decode(ByteBuf in) throws Exception
   {
      List<ByteBuf> out = new ArrayList<>();
      try
      {
         switch (currentState)
         {
            case INIT_BLOCK:
               if (in.readableBytes() < HEADER_LENGTH)
               {
                  break;
               }
               final long magic = in.readLong();
               if (magic != MAGIC_NUMBER)
               {
                  throw new DecompressionException("unexpected block identifier");
               }

               final int token = in.readByte();
               final int compressionLevel = (token & 0x0F) + COMPRESSION_LEVEL_BASE;
               int blockType = token & 0xF0;

               int compressedLength = Integer.reverseBytes(in.readInt());
               if (compressedLength < 0 || compressedLength > MAX_BLOCK_SIZE)
               {
                  throw new DecompressionException(String.format("invalid compressedLength: %d (expected: 0-%d)", compressedLength, MAX_BLOCK_SIZE));
               }

               int decompressedLength = Integer.reverseBytes(in.readInt());
               final int maxDecompressedLength = 1 << compressionLevel;
               if (decompressedLength < 0 || decompressedLength > maxDecompressedLength)
               {
                  throw new DecompressionException(String.format("invalid decompressedLength: %d (expected: 0-%d)", decompressedLength, maxDecompressedLength));
               }
               if (decompressedLength == 0 && compressedLength != 0 || decompressedLength != 0 && compressedLength == 0
                     || blockType == BLOCK_TYPE_NON_COMPRESSED && decompressedLength != compressedLength)
               {
                  throw new DecompressionException(String.format("stream corrupted: compressedLength(%d) and decompressedLength(%d) mismatch",
                                                                 compressedLength,
                                                                 decompressedLength));
               }

               int currentChecksum = Integer.reverseBytes(in.readInt());
               if (decompressedLength == 0 && compressedLength == 0)
               {
                  if (currentChecksum != 0)
                  {
                     throw new DecompressionException("stream corrupted: checksum error");
                  }
                  currentState = State.FINISHED;
                  decompressor = null;
                  checksum = null;
                  break;
               }

               this.blockType = blockType;
               this.compressedLength = compressedLength;
               this.decompressedLength = decompressedLength;
               this.currentChecksum = currentChecksum;

               currentState = State.DECOMPRESS_DATA;
               // fall through
            case DECOMPRESS_DATA:
               blockType = this.blockType;
               compressedLength = this.compressedLength;
               decompressedLength = this.decompressedLength;
               currentChecksum = this.currentChecksum;

               if (in.readableBytes() < compressedLength)
               {
                  break;
               }

               final ByteBufChecksum checksum = this.checksum;
               ByteBuf uncompressed = null;

               try
               {
                  switch (blockType)
                  {
                     case BLOCK_TYPE_NON_COMPRESSED:
                        // Just pass through, we not update the readerIndex yet as we do this outside of the
                        // switch statement.
                        uncompressed = in.retainedSlice(in.readerIndex(), decompressedLength);
                        break;
                     case BLOCK_TYPE_COMPRESSED:
                        uncompressed = ALLOCATOR.buffer(decompressedLength, decompressedLength);

                        decompressor.decompress(CompressionUtil.safeNioBuffer(in),
                                                uncompressed.internalNioBuffer(uncompressed.writerIndex(), decompressedLength));
                        // Update the writerIndex now to reflect what we decompressed.
                        uncompressed.writerIndex(uncompressed.writerIndex() + decompressedLength);
                        break;
                     default:
                        throw new DecompressionException(String.format("unexpected blockType: %d (expected: %d or %d)",
                                                                       blockType,
                                                                       BLOCK_TYPE_NON_COMPRESSED,
                                                                       BLOCK_TYPE_COMPRESSED));
                  }
                  // Skip inbound bytes after we processed them.
                  in.skipBytes(compressedLength);

                  if (checksum != null)
                  {
                     CompressionUtil.checkChecksum(checksum, uncompressed, currentChecksum);
                  }
                  out.add(uncompressed);
                  uncompressed = null;
                  currentState = State.INIT_BLOCK;
               }
               catch (LZ4Exception e)
               {
                  throw new DecompressionException(e);
               }
               finally
               {
                  if (uncompressed != null)
                  {
                     uncompressed.release();
                  }
               }
               break;
            case FINISHED:
            case CORRUPTED:
               in.skipBytes(in.readableBytes());
               break;
            default:
               throw new IllegalStateException();
         }
      }
      catch (Exception e)
      {
         currentState = State.CORRUPTED;
         throw e;
      }

      return out;
   }

   public State getCurrentState()
   {
      return currentState;
   }

   /**
    * Returns {@code true} if and only if the end of the compressed stream has been reached.
    */
   public boolean isClosed()
   {
      return currentState == State.FINISHED;
   }
}
