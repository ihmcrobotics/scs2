package us.ihmc.scs2.session.log;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

public class TimestampScrubberTest
{
   @Test
   void selectsClosestRobotTimestamp(@TempDir File tempDir) throws IOException
   {
      File timestampFile = new File(tempDir, "timestamps.dat");
      Files.writeString(timestampFile.toPath(), """
            1
            60
            1000 10
            2000 20
            3000 30
            """);

      TimestampScrubber scrubber = new TimestampScrubber(timestampFile, true, false);

      assertEquals(10, scrubber.getVideoTimestampFromRobotTimestamp(1500));
      assertEquals(20, scrubber.getVideoTimestampFromRobotTimestamp(1800));
      assertEquals(30, scrubber.getVideoTimestampFromRobotTimestamp(2600));
   }
}
