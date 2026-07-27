package us.ihmc.scs2.session.mcap.encoding;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.lz4.LZ4FFrameInfo;
import org.bytedeco.lz4.LZ4FPreferences;
import org.bytedeco.lz4.global.lz4;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class LZ4FrameDecoderTest
{
   /**
    * {@code LZ4FrameDecoderCompressedData.bin} is real LZ4-compressed MCAP chunk data (independent blocks), stored raw rather than as a decimal
    * text dump to keep the repo small.
    */
   @Test
   public void testWithMCAPSample() throws Exception
   {
      byte[] compressedData = getClass().getResourceAsStream("LZ4FrameDecoderCompressedData.bin").readAllBytes();

      byte[] decompressedBuffer = new LZ4FrameDecoder().decode(compressedData, null);
      // The 2103387 is the expected size of the decompressed buffer
      int expectedLength = 2103387;
      assertEquals(expectedLength, decompressedBuffer.length);
   }

   /**
    * SCS2's own {@link LZ4FrameEncoder} always writes independent blocks, so it can never produce a frame that exercises the bug this test guards
    * against: LZ4 frames using dependent/linked blocks (e.g. produced by ROS2's {@code ros2 bag record} mcap storage plugin, which defaults to
    * linked blocks) used to fail with "Dependent block stream is unsupported" under the old hand-rolled decoder. This test builds a linked-block
    * frame directly via bytedeco's native encoder and confirms {@link LZ4FrameDecoder} -- backed by the real liblz4 reference decompressor -- can
    * read it.
    */
   @Test
   public void testDecodeDependentBlockFrame()
   {
      Random random = new Random(4242L);

      // Use enough data to span multiple blocks (block size below is set to the minimum, 64KB) so linked-block chaining is actually exercised.
      final int LARGE_ENOUGH_SIZE = 200_000;
      byte[] originalData = new byte[LARGE_ENOUGH_SIZE];
      random.nextBytes(originalData);

      byte[] compressedData = compressWithLinkedBlocks(originalData);

      byte[] decompressedData = new LZ4FrameDecoder().decode(compressedData, null);

      assertArrayEquals(originalData, decompressedData);
   }

   /**
    * Feeding {@link LZ4FrameDecoder#decompress(ByteBuffer, int)} a frame that's cut off mid-stream drives it into the exact situation its progress guard exists
    * for: once the truncated input is fully consumed but the frame isn't finished, every further native call is offered zero source bytes, so it
    * can consume nothing and produce nothing (a call can't decompress bytes it was never given) while still reporting "not done yet". Without the
    * guard, that call would just be repeated forever; this test's job is to prove it throws a clear exception instead of hanging.
    */
   @Test
   @Timeout(value = 5, unit = TimeUnit.SECONDS) // Fail instead of hanging the build if we try to decode and end up in an infinite loop.
   public void testDecodeTruncatedFrameThrows()
   {
      Random random = new Random(99L);

      // Large enough that the compressed frame spans multiple LZ4F_decompress calls, so truncating mid-stream (not mid-header) is guaranteed.
      final int LARGE_ENOUGH_SIZE = 200_000;
      byte[] originalData = new byte[LARGE_ENOUGH_SIZE];
      random.nextBytes(originalData);

      byte[] compressedData = new LZ4FrameEncoder().encode(originalData, null);
      byte[] truncatedData = Arrays.copyOf(compressedData, compressedData.length / 2);

      IllegalStateException exception = assertThrows(IllegalStateException.class, () -> new LZ4FrameDecoder().decode(truncatedData, null));
      assertTrue(exception.getMessage().contains("Truncated or corrupt LZ4 frame"),
                 "Expected the truncated-frame message, got: " + exception.getMessage());
   }

   private static byte[] compressWithLinkedBlocks(byte[] data)
   {
      LZ4FFrameInfo frameInfo = new LZ4FFrameInfo();
      frameInfo.blockSizeID(lz4.LZ4F_max64KB);
      frameInfo.blockMode(lz4.LZ4F_blockLinked);
      frameInfo.contentChecksumFlag(lz4.LZ4F_noContentChecksum);
      frameInfo.frameType(lz4.LZ4F_frame);
      frameInfo.contentSize(0);
      frameInfo.dictID(0);
      frameInfo.blockChecksumFlag(lz4.LZ4F_noBlockChecksum);

      LZ4FPreferences preferences = new LZ4FPreferences();
      preferences.frameInfo(frameInfo);
      preferences.compressionLevel(0);
      preferences.autoFlush(0);
      preferences.favorDecSpeed(0);
      preferences.reserved(0, 0);
      preferences.reserved(1, 0);
      preferences.reserved(2, 0);

      ByteBuffer src = ByteBuffer.allocateDirect(data.length);
      src.put(data);
      src.flip();

      long bound = lz4.LZ4F_compressFrameBound(data.length, preferences);
      ByteBuffer dst = ByteBuffer.allocateDirect((int) bound);

      long compressedSize = lz4.LZ4F_compressFrame(new Pointer(dst), bound, new Pointer(src), data.length, preferences);
      if (lz4.LZ4F_isError(compressedSize) != 0)
         throw new RuntimeException(lz4.LZ4F_getErrorName(compressedSize).getString());

      dst.limit((int) compressedSize);
      byte[] result = new byte[dst.remaining()];
      dst.get(result);
      return result;
   }
}
