package us.ihmc.scs2.sessionVisualizer.jfx.session;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.VideoDataReader;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.Objects;

public class TimestampScrubbingTest
{
    private VideoDataReader.ExtendedVideoDataReader reader;

    private long[] actualRobotTimestamps;
    private long[] actualVideoTimestamps;

    @BeforeEach
    public void loadLogDirectory() throws URISyntaxException, IOException
    {
        // GStreamer Algorithm Log
        File dataDirectory = new File(Objects.requireNonNull(getClass().getClassLoader().getResource("sessionLogs/GStreamer_Capture")).toURI());

        LogPropertiesReader logPropertiesReader = new LogPropertiesReader(new File(dataDirectory + "/robotData.log"));
        // Need to have one video in the log or this will fail
        Camera camera = logPropertiesReader.getCameras().get(0);
        reader = new VideoDataReader.ExtendedVideoDataReader(camera, dataDirectory, true);

        // Need to have one video in the log or this will fail
        actualRobotTimestamps = reader.getRobotTimestamps();
        actualVideoTimestamps = reader.getVideoTimestamps();
    }

    @Test
    public void testGoingThroughRobotTimestampsInOrder()
    {
        // Go through the robot timestamps in order and see if we get the desired video timestamp
        for (int i = 0; i < actualRobotTimestamps.length / 12; i++)
        {
            reader.readVideoFrame(actualRobotTimestamps[i]);

            // Skip first frame since it's often messed up at the beginning
            if (i < 1)
                continue;

            Assertions.assertEquals(reader.getCurrentVideoTimestamp(), actualVideoTimestamps[i]);
        }
    }

    @Test
    public void testGoingThroughRobotTimestampsEveryOther()
    {
        // Go through the robot timestamps by +=2, so we skip every other frame and see if we get the desired video timestamp
        for (int i = 0; i < actualRobotTimestamps.length / 8; i+=2)
        {
            reader.readVideoFrame(actualRobotTimestamps[i]);

            // Skip first frame since it's often messed up at the beginning
            if (i < 1)
                continue;

            Assertions.assertEquals(reader.getCurrentVideoTimestamp(), actualVideoTimestamps[i]);
        }
    }

    @Test
    public void testGettingRandomTimestamp()
    {
        // Test grabbing random robot timestamps and checking to make sure we get the correct video timestamp
        reader.readVideoFrame(actualRobotTimestamps[26]);
        Assertions.assertEquals(reader.getCurrentVideoTimestamp(), actualVideoTimestamps[26]);

        reader.readVideoFrame(actualRobotTimestamps[40]);
        Assertions.assertEquals(reader.getCurrentVideoTimestamp(), actualVideoTimestamps[40]);

        reader.readVideoFrame(actualRobotTimestamps[34]);
        Assertions.assertEquals(reader.getCurrentVideoTimestamp(), actualVideoTimestamps[34]);
    }

    @Test
    public void testGoingThroughRobotTimestampsBackwards()
    {
        for (int i = actualRobotTimestamps.length / 8; i > 0; i--)
        {
            reader.readVideoFrame(actualRobotTimestamps[i]);
            Assertions.assertEquals(reader.getCurrentVideoTimestamp(), actualVideoTimestamps[i]);
        }
    }
}
