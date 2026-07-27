package us.ihmc.scs2.session.mcap.encoding;

import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class LZ4FrameEncodingTest
{
   @Test
   public void testEncodeDecode()
   {
      Random random = new Random(23423L);

      for (int i = 0; i < 100; i++)
      {
         byte[] originalData = new byte[random.nextInt(1000) + 10];
         random.nextBytes(originalData);

         LZ4FrameEncoder lz4Compressor = new LZ4FrameEncoder();

         byte[] compressedData = lz4Compressor.encode(originalData, null);

         LZ4FrameDecoder lz4Decoder = new LZ4FrameDecoder();
         byte[] decompressedData = lz4Decoder.decode(compressedData, null);

         assertArrayEquals(originalData, decompressedData);
      }
   }

   /**
    * {@link LZ4FrameEncoder} is configured for 4MB blocks (see its constructor), but every other test in this class stays well under that, so
    * the multi-block chunking path -- what happens when content actually has to be split across more than one block -- is never exercised
    * elsewhere. This forces that path with a payload larger than one block.
    */
   @Test
   public void testEncodeDecodeMultiBlock()
   {
      Random random = new Random(778899L);

      byte[] originalData = new byte[5 * 1024 * 1024]; // 5MB > the encoder's 4MB block size
      random.nextBytes(originalData);

      byte[] compressedData = new LZ4FrameEncoder().encode(originalData, null);
      byte[] decompressedData = new LZ4FrameDecoder().decode(compressedData, null);

      assertArrayEquals(originalData, decompressedData);
   }

   /**
    * Every other test in this class only exercises inOffset/outOffset == 0 via the 2-arg convenience overloads. This drives the 4-arg
    * encode(...)/decode(...) overloads directly with nonzero offsets on both sides, with random padding surrounding the real payload on the
    * input side, to confirm the offset/length parameters are actually respected rather than the whole array being (mis)used.
    */
   @Test
   public void testEncodeDecodeWithOffsets()
   {
      Random random = new Random(112233L);

      // inOffset/outOffset are arbitrary but deliberately nonzero, non-round, and different from each other -- if the implementation ever
      // confused the two offsets, or silently fell back to treating them as 0, using the same "nice" number for both could hide that.
      int inOffset = 7;
      int dataLength = 500;
      byte[] originalData = new byte[dataLength];
      random.nextBytes(originalData);

      // Build an input array where the real payload sits in the middle of unrelated data: inOffset bytes of padding before it, 13 more
      // (an arbitrary nonzero amount) after it. Filling the whole array with random bytes first -- not zeros -- and only then overwriting
      // the [inOffset, inOffset + dataLength) slice with originalData means the padding is genuinely indistinguishable from real content.
      // That matters: if encode(...) ignored inOffset/inLength and just compressed the whole array, this test would fail loudly instead of
      // accidentally passing because the "ignored" bytes happened to be zero.
      int extraPaddingAtEnd = 13;
      byte[] paddedInput = new byte[inOffset + dataLength + extraPaddingAtEnd];
      random.nextBytes(paddedInput);
      // Read the parameters in System.arraycopy() to understand this
      System.arraycopy(originalData, 0, paddedInput, inOffset, dataLength);

      // Compress only the real payload slice; the leading/trailing padding in paddedInput must not end up in the compressed output.
      byte[] compressed = new LZ4FrameEncoder().encode(paddedInput, inOffset, dataLength, null, 0);

      // Mirror the same idea on the decode side, with its own distinct offset: decode(...) should write the decompressed bytes starting at
      // outOffset rather than at the start of the array. paddedOutput is sized to fit exactly (no slack after the payload), since decode(...)
      // is expected to write precisely dataLength bytes there and nothing past it.
      int outOffset = 9;
      byte[] paddedOutput = new byte[outOffset + dataLength];
      // decode(byte[], byte[]) hands back the same array reference it was given when out != null (see LZ4FrameDecoder), so result == paddedOutput
      // here; it's reassigned to a new variable only to make that contract explicit at the call site.
      byte[] result = new LZ4FrameDecoder().decode(compressed, 0, compressed.length, paddedOutput, outOffset);

      // result is the whole backing array, including the outOffset bytes of padding before the real content -- slice it out before
      // comparing against originalData.
      // Docs for Arrays.copyOfRange() explain what's going on here nicely
      byte[] decoded = Arrays.copyOfRange(result, outOffset, outOffset + dataLength);
      assertArrayEquals(originalData, decoded);
   }
}
