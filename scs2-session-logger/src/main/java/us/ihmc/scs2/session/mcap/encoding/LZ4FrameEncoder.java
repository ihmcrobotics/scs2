package us.ihmc.scs2.session.mcap.encoding;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.lz4.LZ4FFrameInfo;
import org.bytedeco.lz4.LZ4FPreferences;
import org.bytedeco.lz4.global.lz4;

import java.nio.ByteBuffer;

/**
 * Encodes LZ4 frame-format streams, backed by bytedeco's JNI bindings to the reference liblz4 implementation ({@code lz4frame.h}).
 * <p>
 * Always writes independent blocks with 4MB block size and no checksums, matching the write behavior of the Java implementation this replaces --
 * so the on-disk format produced here is unchanged. Only {@link LZ4FrameDecoder} gains new read capability (dependent/linked blocks).
 * </p>
 */
public class LZ4FrameEncoder
{
   // Built once and reused for every encode() call on this instance -- LZ4FPreferences is just a read-only settings struct handed to each
   // LZ4F_compressFrame call, it holds no per-call state, so there's no reason to rebuild it every time.
   private final LZ4FPreferences preferences;

   public LZ4FrameEncoder()
   {
      // bytedeco ships LZ4F_INIT_FRAMEINFO()/LZ4F_INIT_PREFERENCES() helpers that are supposed to fill these structs with sane defaults, but
      // they're not actually present in this version's compiled jar (checked via javap), so every field is set explicitly here instead of
      // relying on a default-initialized struct -- native memory isn't guaranteed to come back zeroed.
      LZ4FFrameInfo frameInfo = new LZ4FFrameInfo();
      // 4MB blocks, matching the old hand-rolled encoder's default (BLOCKSIZE.SIZE_4MB) -- larger blocks compress MCAP chunk-sized payloads
      // better than the smaller options (64KB/256KB/1MB).
      frameInfo.blockSizeID(lz4.LZ4F_max4MB);
      // Independent blocks, matching the old encoder's only supported mode. This is a deliberate compatibility choice, not just parity: linked
      // (dependent) blocks compress slightly better, but plenty of decoders -- including the one this class used to be paired with -- can't
      // read them. Writing independent blocks means anything SCS2 produces stays readable everywhere, even though SCS2's own LZ4FrameDecoder
      // can now read either mode.
      frameInfo.blockMode(lz4.LZ4F_blockIndependent);
      // No content/block checksums, matching the old encoder's defaults -- MCAP chunks already carry their own CRC32 of the uncompressed
      // records (see MutableChunk.uncompressedCRC32()), so an LZ4-level checksum would just be redundant integrity-checking at extra CPU/byte
      // cost.
      frameInfo.contentChecksumFlag(lz4.LZ4F_noContentChecksum);
      frameInfo.blockChecksumFlag(lz4.LZ4F_noBlockChecksum);
      frameInfo.frameType(lz4.LZ4F_frame);
      frameInfo.contentSize(0); // 0 = "not declaring the uncompressed size up front", same as the old encoder never set FLG.Bits.CONTENT_SIZE.
      frameInfo.dictID(0); // Not using a shared/external compression dictionary.

      preferences = new LZ4FPreferences();
      preferences.frameInfo(frameInfo);
      preferences.compressionLevel(0); // 0 = default (fast) compression level, not the slower high-compression mode.
      preferences.autoFlush(0);
      preferences.favorDecSpeed(0);
      // Must be zero for forward compatibility per the LZ4F_preferences_t spec -- again, explicit because we can't assume the native
      // allocation came back zeroed.
      preferences.reserved(0, 0);
      preferences.reserved(1, 0);
      preferences.reserved(2, 0);
   }

   public byte[] encode(byte[] in, byte[] out)
   {
      return encode(in, 0, in.length, out, 0);
   }

   public byte[] encode(byte[] in, int inOffset, int inLength, byte[] out, int outOffset)
   {
      ByteBuffer inBuffer = ByteBuffer.wrap(in);
      ByteBuffer outBuffer = out == null ? null : ByteBuffer.wrap(out);
      ByteBuffer result = encode(inBuffer, inOffset, inLength, outBuffer, outOffset);
      return result.array();
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

      try
      {
         ByteBuffer srcDirect = LZ4NativeUtil.toDirect(in);

         long bound = lz4.LZ4F_compressFrameBound(inLength, preferences);
         ByteBuffer dstDirect = ByteBuffer.allocateDirect((int) bound);

         Pointer srcPointer = new Pointer(srcDirect);
         Pointer dstPointer = new Pointer(dstDirect);

         long compressedSize = lz4.LZ4F_compressFrame(dstPointer, bound, srcPointer, inLength, preferences);
         LZ4NativeUtil.checkError(compressedSize);

         dstDirect.limit((int) compressedSize);

         if (out != null)
         {
            out.position(outOffset);
            out.put(dstDirect);
            out.flip();
            return out;
         }
         else
         {
            ByteBuffer result = ByteBuffer.allocate((int) compressedSize);
            result.put(dstDirect);
            result.flip();
            return result;
         }
      }
      finally
      {
         in.limit(limitPrev);
      }
   }
}
