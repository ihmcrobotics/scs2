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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

import io.netty.handler.codec.compression.DecompressionException;
import net.jpountz.lz4.LZ4Compressor;
import net.jpountz.lz4.LZ4DecompressorWithLength;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import net.jpountz.xxhash.XXHash32;
import net.jpountz.xxhash.XXHashFactory;

/**
 * https://github.com/lz4/lz4/blob/dev/doc/lz4_Frame_format.md
 */
public class Lz4FrameDecoder
{
   /**
    * Magic number of LZ4 block: "4 Bytes, Little endian format. Value : 0x184D2204"
    */
   private static final int MAGIC_NUMBER = 0x184D2204;
   /**
    * Default seed value for xxhash.
    */
   static final int DEFAULT_SEED = 0;

   private static final XXHash32 XXHASH32 = XXHashFactory.fastestInstance().hash32();

   private static final int MIN_HEADER_LENGTH = 7;

   /**
    * Underlying decompressor in use.
    */
   private LZ4FastDecompressor decompressor;

   private final boolean validateChecksums;

   public Lz4FrameDecoder()
   {
      this(false);
   }

   public Lz4FrameDecoder(boolean validateChecksums)
   {
      this(LZ4Factory.fastestInstance(), validateChecksums);
   }

   public Lz4FrameDecoder(LZ4Factory factory, boolean validateChecksums)
   {
      this.validateChecksums = validateChecksums;
      decompressor = factory.fastDecompressor();
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
      FrameDescriptor frameDescriptor = new FrameDescriptor();
      frameDescriptor.read(in);

      if (frameDescriptor.flgByte.dictionaryIDFlag)
         throw new DecompressionException("Dictionary ID is supported yet");

      if (validateChecksums)
      {
         int hash = XXHASH32.hash(in, 0, frameDescriptor.frameDescriptorLength, DEFAULT_SEED);
         if (((hash >> 8) & 0xFF) != frameDescriptor.headChecksum)
            throw new DecompressionException("Header checksum failed.");
      }

      return out;
   }

   /**
    * <a href="https://github.com/lz4/lz4/blob/dev/doc/lz4_Frame_format.md#frame-descriptor">Frame
    * Descriptor</a>
    */
   private static class FrameDescriptor
   {
      private final FLGByte flgByte = new FLGByte();
      private final BDByte bdByte = new BDByte();

      /**
       * This is the original (uncompressed) size. This information is optional, and only present if the
       * associated flag is set. Content size is provided using unsigned 8 Bytes, for a maximum of 16
       * Exabytes. Format is Little endian. This value is informational, typically for display or memory
       * allocation. It can be skipped by a decoder, or used to validate content correctness.
       */
      long contentSize = -1;
      /**
       * If this flag is set, a 4-bytes Dict-ID field will be present, after the descriptor flags and the
       * Content Size.
       */
      int dictionaryID;
      /**
       * One-byte checksum of combined descriptor fields, including optional ones. The value is the second
       * byte of xxh32() : (xxh32()>>8) & 0xFF using zero as a seed, and the full Frame Descriptor as an
       * input (including optional fields when they are present). A wrong checksum indicates that the
       * descriptor is erroneous.
       */
      byte headChecksum;
      int frameDescriptorLength;

      public void read(ByteBuffer in)
      {
         in.order(ByteOrder.LITTLE_ENDIAN);
         flgByte.readFlags(in.get());
         frameDescriptorLength++;
         bdByte.read(in.get());
         frameDescriptorLength++;

         if (flgByte.contentChecksumFlag)
         {
            contentSize = in.getLong();
            frameDescriptorLength += 8;
         }
         else
         {
            contentSize = -1;
         }

         if (flgByte.dictionaryIDFlag)
         {
            dictionaryID = in.getInt();
         }
         else
         {
            dictionaryID = -1;
         }

         headChecksum = in.get();
         frameDescriptorLength++;
      }

      /**
       * <a href="https://github.com/lz4/lz4/blob/dev/doc/lz4_Frame_format.md#frame-descriptor">Frame
       * Descriptor - FLG byte</a>
       */
      private static class FLGByte
      {
         /**
          * A byte used to store flags as follows:
          * <ul>
          * <li>bit 0: dictionary ID flag.
          * <li>bit 1: reserved for future expansion.
          * <li>bit 2: content checksum flag.
          * <li>bit 3: content size flag.
          * <li>bit 4: block checksum flag.
          * <li>bit 5: block independence flag.
          * <li>bits 6-7: version, should be "01".
          * </ul>
          */
         private byte flg;
         /**
          * If this flag is set, a 4-bytes Dict-ID field will be present, after the descriptor flags and the
          * Content Size.
          */
         private boolean dictionaryIDFlag;
         /**
          * If this flag is set, a 32-bits content checksum will be appended after the EndMark.
          */
         private boolean contentChecksumFlag;
         /**
          * If this flag is set, the uncompressed size of data included within the frame will be present as
          * an 8 bytes unsigned little endian value, after the flags. Content Size usage is optional.
          */
         private boolean contentSizeFlag;
         /**
          * If this flag is set, each data block will be followed by a 4-bytes checksum, calculated by using
          * the xxHash-32 algorithm on the raw (compressed) data block. The intention is to detect data
          * corruption (storage or transmission errors) immediately, before decoding. Block checksum usage is
          * optional.
          */
         private boolean blockChecksumFlag;
         private boolean blockIndependenceFlag;
         /**
          * 2-bits field, must be set to 01. Any other value cannot be decoded by this version of the
          * specification. Other version numbers will use different flag layouts.
          */
         private int version;

         public void readFlags(byte flg)
         {
            this.flg = flg;
            dictionaryIDFlag = (flg & 1) == 1;
            contentChecksumFlag = ((flg >> 2) & 1) == 1;
            contentSizeFlag = ((flg >> 3) & 1) == 1;
            blockChecksumFlag = ((flg >> 4) & 1) == 1;
            blockIndependenceFlag = ((flg >> 5) & 1) == 1;
            version = flg >> 6;
         }
      }

      /**
       * <a href="https://github.com/lz4/lz4/blob/dev/doc/lz4_Frame_format.md#frame-descriptor">Frame
       * Descriptor - BD byte</a>
       */
      private static class BDByte
      {
         /**
          * A byte used to store the block max size as follows:
          * <ul>
          * <li>bit 7: reserved for future expansion.
          * <li>bits 6-5-4: block maximum size.
          * <li>bits 3-2-1-0: reserved for future expansion.
          * </ul>
          */
         private byte bd;
         /**
          * This information is useful to help the decoder allocate memory. Size here refers to the original
          * (uncompressed) data size. Block Maximum Size is one value among the following table :
          * <ul>
          * <li>0 through 3: N/A
          * <li>4: 64 KB
          * <li>5: 256 KB
          * <li>6: 1 MB
          * <li>7: 4 MB
          * </ul>
          */
         private int blockMaximumSize;

         public void read(byte bd)
         {
            this.bd = bd;
            blockMaximumSize = (bd >> 4) - ((bd >> 4) & 4);
            if (blockMaximumSize == 4)
               blockMaximumSize = 64000; // 64KB
            else if (blockMaximumSize == 5)
               blockMaximumSize = 256000; // 256KB
            else if (blockMaximumSize == 6)
               blockMaximumSize = 1000000; // 1MB
            else if (blockMaximumSize == 7)
               blockMaximumSize = 4000000; // 4MB
         }
      }
   }

   private static class DataBlock
   {
      /**
       * <p>
       * This field uses 4-bytes, format is little-endian.
       * </p>
       * <p>
       * If the highest bit is set (1), the block is uncompressed.
       * </p>
       * <p>
       * All other bits give the size, in bytes, of the data section. The size does not include the block
       * checksum if present.
       * </p>
       * <p>
       * Block_Size shall never be larger than Block_Maximum_Size. Such an outcome could potentially
       * happen for non-compressible sources. In such a case, such data block must be passed using
       * uncompressed format.
       * </p>
       * <p>
       * A value of 0x00000000 is invalid, and signifies an EndMark instead. Note that this is different
       * from a value of 0x80000000 (highest bit set), which is an uncompressed block of size 0 (empty),
       * which is valid, and therefore doesn't end a frame. Note that, if Block_checksum is enabled, even
       * an empty block must be followed by a 32-bit block checksum.
       * </p>
       */
      private int blockSize;
      private boolean isBlockCompressed;
      /**
       * <p>
       * Where the actual data to decode stands. It might be compressed or not, depending on previous
       * field indications.
       * </p>
       * <p>
       * When compressed, the data must respect the LZ4 block format specification.
       * </p>
       * <p>
       * Note that a block is not necessarily full. Uncompressed size of data can be any size up to
       * Block_Maximum_Size, so it may contain less data than the maximum block size.
       * </p>
       */
      private ByteBuffer data;
      /**
       * <p>
       * Only present if the associated flag is set. This is a 4-bytes checksum value, in little endian
       * format, calculated by using the xxHash-32 algorithm on the raw (undecoded) data block, and a seed
       * of zero. The intention is to detect data corruption (storage or transmission errors) before
       * decoding.
       * </p>
       * <p>
       * Block_checksum can be cumulative with Content_checksum.
       * </p>
       */
      private int blockChecksum;

      public void read(ByteBuffer in, int blockMaximumSize, boolean blockChecksumFlag, LZ4FastDecompressor decompressor)
      {
         byte bs0 = in.get();
         byte bs1 = in.get();
         byte bs2 = in.get();
         byte bs3 = in.get();
         isBlockCompressed = ((bs3 >> 7) & 1) == 0;
         bs3 = (byte) (((byte) (bs3 << 1)) >> 1); // Drop the highest bit to get the actual size
         blockSize = ((bs3 << 24) & 0xFF) | ((bs2 << 16) & 0xFF) | ((bs1 << 8) & 0xFF) | ((bs0 << 0) & 0xFF);

         int pos = in.position();

         if (data == null || data.capacity() < blockMaximumSize)
            data = ByteBuffer.allocate(blockMaximumSize);
         else
            data.rewind();

         if (isBlockCompressed)
         {
            LZ4DecompressorWithLength.getDecompressedLength(in, srcOff)
            decompressor.decompress(in, 0, data, 0, blockSize);
         }
         else
            data.put(0, in, 0, blockSize);
         // Reset markers for reading
         data.limit(blockSize);
         data.position(0);

         in.position(pos + blockSize);

         if (blockChecksumFlag)
         {
            blockChecksum = in.getInt();
         }
         else
         {
            blockChecksum = -1;
         }
      }
   }

   public static void main(String[] args)
   {
      LZ4Factory factory = LZ4Factory.fastestInstance();
      LZ4Compressor compressor = factory.fastCompressor();
      LZ4FastDecompressor decompressor = factory.fastDecompressor();

      String expectedStr = "Yooooooooooooooo! Ca va bien ou quoi?!";
      ByteBuffer source = ByteBuffer.wrap(expectedStr.getBytes());
      System.out.println("Source size: " + source.remaining());
      ByteBuffer compressed = ByteBuffer.allocate(expectedStr.getBytes().length * 2);
      ByteBuffer decompressed = ByteBuffer.allocate(expectedStr.getBytes().length * 2);

      compressor.compress(source, compressed);
      compressed.flip();
      System.out.println("Compressed size: " + compressed.remaining());

      int length = LZ4DecompressorWithLength.getDecompressedLength(compressed);
      System.out.println("Decompressed estimated size: " + length);
      compressed.position(0);
      decompressor.decompress(compressed, 0, decompressed, 0, length);

      String actualStr = new String(decompressed.array());
      System.out.println(actualStr);
   }
}
