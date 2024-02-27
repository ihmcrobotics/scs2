package us.ihmc.scs2.session.mcap.encoding;

import gnu.trove.list.array.TByteArrayList;
import org.junit.jupiter.api.Test;

import java.util.Scanner;

public class LZ4FrameDecoderTest
{

   @Test
   public void testWithMCAPSample() throws Exception
   {
      TByteArrayList byteList = new TByteArrayList();
      Scanner scanner = new Scanner(getClass().getResourceAsStream("LZ4FrameDecoderCompressedData.txt"));
      scanner.useDelimiter(", ");

      while (scanner.hasNextByte())
         byteList.add((byte) scanner.nextByte());
      scanner.close();

      byte[] decompressedBuffer = new LZ4FrameDecoder().decode(byteList.toArray(), null);
      System.out.println(decompressedBuffer.length);
   }
}
