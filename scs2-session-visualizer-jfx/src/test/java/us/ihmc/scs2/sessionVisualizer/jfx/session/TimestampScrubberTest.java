package us.ihmc.scs2.sessionVisualizer.jfx.session;

import org.apache.commons.math.stat.descriptive.moment.StandardDeviation;
import org.junit.jupiter.api.*;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.VideoDataReader;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests in this class are disabled because the files have duplicate timestamps that will cause issues when trying to retrieve a specific timestamp
 */

public class TimestampScrubberTest
{
    private static VideoDataReader.TimestampScrubber scrubber;

    private static long[] actualRobotTimestamps;
    private static long[] actualVideoTimestamps;

    // After the controller stops we generate a lot of garbage timestamps. This prevents us from trying to use them
    private static int duplicatesAtEndOfFile = 1;

    @BeforeAll
    public static void loadFileTimestamps() throws URISyntaxException, IOException
    {
        File timestampFile = new File("//10.7.4.48/LogData/Nadia/20230427_NadiaRunning/20230427_183903_NadiaRunningTallerCompleteFailRobotBreakMaybe/NadiaPoleNorth_Timestamps.dat");
//        File timestampFile = new File(Objects.requireNonNull(TimestampScrubberTest.class.getClassLoader().getResource("sessionLogs/GStreamer_HDMI_timestamps_100.dat")).toURI());

        scrubber = new VideoDataReader.TimestampScrubber(timestampFile, true, false);

        actualRobotTimestamps = scrubber.getRobotTimestampsArray();
        actualVideoTimestamps = scrubber.getVideoTimestampsArray();

        // Check for duplicate robotTimestamps at end of file. Due to delay between controller and logger we don't want to even consider those indexes
        for (int i = actualRobotTimestamps.length - 1; i > 0; i--)
        {
            if (actualRobotTimestamps[i] == actualRobotTimestamps[i - 1])
                duplicatesAtEndOfFile++;
        }
        // Need to have one video in the log or this will fail
    }

    @Test
    public void testChronologicallyIncreasingRobotTimestamps()
    {
        // Go through the robot timestamps in order to make sure they are in order
        for (int i = 1; i < actualRobotTimestamps.length - duplicatesAtEndOfFile; i++)
        {
            long previousTimestamp = actualRobotTimestamps[i - 1];
            long currentTimestamp = actualRobotTimestamps[i];

            // Useful for debugging timestamps issues, won't print anything if the checkForDuplicates() is run inside the VideoDataReader
            if (currentTimestamp == previousTimestamp)
                System.out.println(currentTimestamp + " -- " + previousTimestamp);

//            Assertions.assertTrue(currentTimestamp > previousTimestamp,
//                    "Cureent: " + currentTimestamp + " and Previous: " + previousTimestamp + " at Index: " + i);
        }
    }

    @Test
    public void testStandardDeviationBetweenRobotTimestamps()
    {
        StandardDeviation standardDeviation = new StandardDeviation();

        double[] copyRobotTimestamps = new double[actualRobotTimestamps.length];

        for (int i = 1; i < actualRobotTimestamps.length; i++)
        {
            long currentDelta = actualRobotTimestamps[i] - actualRobotTimestamps[i - 1];
            copyRobotTimestamps[i - 1] = currentDelta;
        }

        System.out.println("robotTimestamps Length: " + actualRobotTimestamps.length);
        System.out.println("copyOfRobotTimestamps Length: " + copyRobotTimestamps.length);
        System.out.println("Standard Deviation: " + (long) standardDeviation.evaluate(copyRobotTimestamps));
    }

    @Test
    public void testDeltaStatisticsBetweenRobotTimestamps()
    {
        long previousTimestamp;
        long delta = 0;
        long smallestDelta = 1000000000;
        long largestDelta = 0;
        int duplicates = 0;
        long whereItHappened = 0;

        // Go through the robot timestamps in order and check the next one is larger
        for (int i = 1; i < actualRobotTimestamps.length - duplicatesAtEndOfFile; i++)
        {
            previousTimestamp = actualRobotTimestamps[i - 1];
            long currentTimestamp = actualRobotTimestamps[i];

            Assertions.assertTrue(currentTimestamp > previousTimestamp,
                    "Cureent: " + currentTimestamp + "\n Previous: " + previousTimestamp);

            long currentDelta = currentTimestamp - previousTimestamp;
            delta += currentDelta;

            if (currentDelta < smallestDelta)
                smallestDelta = currentDelta;

            if (currentDelta > largestDelta)
            {
                largestDelta = currentDelta;
                whereItHappened = previousTimestamp;
            }
        }

        delta = delta / (actualRobotTimestamps.length - duplicates);

        System.out.println("Smallest Delta: " + smallestDelta);
        System.out.println("Largest Delta: " + largestDelta);
        System.out.println("Duplicate robotTimestamps: " + duplicates);
        System.out.println("Average delta for robotTimestamps: " + delta);
        System.out.println("Where it happened: " + whereItHappened);
    }

    @Test
    public void testGoingThroughRobotTimestampsInOrder()
    {
        // Go through the robot timestamps in order and see if we get the desired video timestamp
        for (int i = 0; i < actualRobotTimestamps.length - duplicatesAtEndOfFile; i++)
        {
            long currentVideoTimestamp = scrubber.getVideoTimestamp(actualRobotTimestamps[i]);
            assertEquals(currentVideoTimestamp, actualVideoTimestamps[i]);
        }
    }

    @Test
    public void testGoingThroughRobotTimestampsBackwards()
    {
        for (int i = actualRobotTimestamps.length - duplicatesAtEndOfFile - 1; i > 0; i--)
        {
            scrubber.getVideoTimestamp(actualRobotTimestamps[i]);
            assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[i]);
        }
    }

    @Test
    public void testGoingThroughRobotTimestampsEveryOther()
    {
        // Go through the robot timestamps by +=2, so we skip every other frame and see if we get the desired video timestamp
        for (int i = 0; i < actualRobotTimestamps.length - duplicatesAtEndOfFile; i+=2)
        {
            scrubber.getVideoTimestamp(actualRobotTimestamps[i]);
            assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[i],
                    "For loop index: " + i);
        }
    }

    @Test
    public void testGettingRandomTimestamp()
    {
        // Test grabbing random robot timestamps and checking to make sure we get the correct video timestamp
        // These robot timestamps need to be unique or the binary search will fail to get the correct video timestamp
        scrubber.getVideoTimestamp(actualRobotTimestamps[26]);
        assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[26]);

        scrubber.getVideoTimestamp(actualRobotTimestamps[40]);
        assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[40]);

        scrubber.getVideoTimestamp(actualRobotTimestamps[34]);
        assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[34]);
    }

    @Test
    public void testSearchRobotTimestampsForIndex()
    {
        // Sets the currentIndex to the end of the
        scrubber.getVideoTimestamp(Long.MAX_VALUE);
        int endOfArray = scrubber.getCurrentIndex();

        assertEquals(actualRobotTimestamps.length - 1, endOfArray);
    }

    @Test
    public void testFileNotFoundException()
    {
        File badName = new File("This_is_a_bad_file_name_lol");

        Throwable thrown = assertThrows(RuntimeException.class, () -> new VideoDataReader.TimestampScrubber(badName, true, false));
        assertEquals("java.io.FileNotFoundException: " + badName + " (The system cannot find the file specified)", thrown.getMessage());
    }

    @Test
    public void testInsertionPointWhenSearching()
    {
        // Trying to find a robotTimestamp that doesn't exist will cause an insertionPoint
        scrubber.getVideoTimestamp(actualRobotTimestamps[0] + 1);
        int insertionPointIndex = scrubber.getCurrentIndex();

        assertEquals(1, insertionPointIndex);
    }
}
