package us.ihmc.scs2.session.mcap.encoding;

import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
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

         // Gonna have to use a ByteArrayOutputStream to comply with the API
         ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();

         LZ4FrameEncoder lz4Compressor = new LZ4FrameEncoder();

         byte[] compressedData = lz4Compressor.encode(originalData, null);

         LZ4FrameDecoder lz4Decoder = new LZ4FrameDecoder();
         byte[] decompressedData = lz4Decoder.decode(compressedData, null);

         assertArrayEquals(originalData, decompressedData);
      }
   }
}
