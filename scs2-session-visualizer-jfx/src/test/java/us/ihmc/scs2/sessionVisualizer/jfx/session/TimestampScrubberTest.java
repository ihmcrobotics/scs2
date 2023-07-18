package us.ihmc.scs2.sessionVisualizer.jfx.session;

import org.apache.commons.math.stat.descriptive.moment.StandardDeviation;
import org.junit.jupiter.api.*;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.BytedecoVideoReader;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.VideoDataReader;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class TimestampScrubberTest
{
    private static BytedecoVideoReader.TimestampScrubber scrubber;

    private static long[] robotTimestamps;
    private static long[] videoTimestamps;

    // After the controller stops we generate a lot of garbage timestamps. This prevents us from trying to use them
    private static int duplicatesAtEndOfFile = 1;

    @BeforeAll
    public static void loadFileTimestamps() throws URISyntaxException, IOException
    {
//        File timestampFile = new File("//10.7.4.48/LogData/Nadia/20230427_NadiaRunning/20230427_183903_NadiaRunningTallerCompleteFailRobotBreakMaybe/NadiaPoleNorth_Timestamps.dat");
        File timestampFile = new File(Objects.requireNonNull(TimestampScrubberTest.class.getClassLoader().getResource("sessionLogs/Capture.dat")).toURI());

        scrubber = new BytedecoVideoReader.TimestampScrubber(timestampFile, true, false);

        // Need to have one video in the log or this will fail
        robotTimestamps = scrubber.getRobotTimestampsArray();
        videoTimestamps = scrubber.getVideoTimestampsArray();

        // Check for duplicate robotTimestamps at end of file. Due to delay between controller and logger we don't want to even consider those indexes
        for (int i = robotTimestamps.length - 1; i > 0; i--)
        {
            if (robotTimestamps[i] == robotTimestamps[i - 1])
                duplicatesAtEndOfFile++;
        }
        // Need to have one video in the log or this will fail
    }

    @Test
    public void testChronologicallyIncreasingRobotTimestamps()
    {
        long previousRobotTimestamp;

        // Go through the robot timestamps in order and check the next one is larger
        for (int i = 1; i < robotTimestamps.length - duplicatesAtEndOfFile; i++)
        {
            previousRobotTimestamp = robotTimestamps[i - 1];
            long currentRobotTimestamp = robotTimestamps[i];

            // Useful for debugging timestamps issues, won't print anything if the checkForDuplicates() is run inside the VideoDataReader
            if (currentRobotTimestamp == previousRobotTimestamp)
                System.out.println(currentRobotTimestamp + " -- " + previousRobotTimestamp);

            Assertions.assertTrue(currentRobotTimestamp > previousRobotTimestamp,
                    "Cureent: " + currentRobotTimestamp + " and Previous: " + previousRobotTimestamp + " at Index: " + i);
        }
    }

    @Test
    public void testStandardDeviationBetweenRobotTimestamps()
    {
        StandardDeviation standardDeviation = new StandardDeviation();

        double[] copyRobotTimestamps = new double[robotTimestamps.length];

        for (int i = 1; i < robotTimestamps.length; i++)
        {
            long currentDelta = robotTimestamps[i] - robotTimestamps[i - 1];
            copyRobotTimestamps[i - 1] = currentDelta;
        }

        System.out.println("robotTimestamps Length: " + robotTimestamps.length);
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
        for (int i = 1; i < robotTimestamps.length - duplicatesAtEndOfFile; i++)
        {
            previousTimestamp = robotTimestamps[i - 1];
            long currentTimestamp = robotTimestamps[i];

            Assertions.assertTrue(currentTimestamp > previousTimestamp,
                    "Current: " + currentTimestamp + "\n Previous: " + previousTimestamp + " at Index: " + i);

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

        delta = delta / (robotTimestamps.length - duplicates);

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
        for (int i = 0; i < robotTimestamps.length - duplicatesAtEndOfFile; i++)
        {
            long currentVideoTimestamp = scrubber.getVideoTimestamp(robotTimestamps[i]);
            assertEquals(currentVideoTimestamp, videoTimestamps[i]);
        }
    }

    @Test
    public void testGoingThroughRobotTimestampsBackwards()
    {
        for (int i = robotTimestamps.length - duplicatesAtEndOfFile - 1; i > 0; i--)
        {
            scrubber.getVideoTimestamp(robotTimestamps[i]);
            assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[i]);
        }
    }

    @Test
    public void testGoingThroughRobotTimestampsEveryOther()
    {
        // Go through the robot timestamps by +=2, so we skip every other frame and see if we get the desired video timestamp
        for (int i = 0; i < robotTimestamps.length - duplicatesAtEndOfFile; i+=2)
        {
            scrubber.getVideoTimestamp(robotTimestamps[i]);
            assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[i],
                    "For loop index: " + i);
        }
    }

    @Test
    public void testGettingRandomTimestamp()
    {
        // Test grabbing random robot timestamps and checking to make sure we get the correct video timestamp
        // These robot timestamps need to be unique or the binary search will fail to get the correct video timestamp
        scrubber.getVideoTimestamp(robotTimestamps[26]);
        assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[26]);

        scrubber.getVideoTimestamp(robotTimestamps[40]);
        assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[40]);

        scrubber.getVideoTimestamp(robotTimestamps[34]);
        assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[34]);
    }

    @Test
    public void testSearchRobotTimestampsForIndex()
    {
        // Sets the currentIndex to the end of the videoTimestamps to test edge case
        scrubber.getVideoTimestamp(Long.MAX_VALUE);
        int endOfArray = scrubber.getCurrentIndex();

        assertEquals(robotTimestamps.length - 1, endOfArray);
    }

    @Test
    public void testFileNotFoundException()
    {
        File badName = new File("This_is_a_bad_file_name_lol");

        Throwable thrown = assertThrows(RuntimeException.class, () -> new VideoDataReader.TimestampScrubber(badName, true, false));
        String messageException = thrown.getMessage().substring(0, 58);

        assertEquals("java.io.FileNotFoundException: " + badName, messageException);
    }

    @Test
    public void testInsertionPointWhenSearching()
    {
        // Trying to find a robotTimestamp that doesn't exist will cause an insertionPoint
        scrubber.getVideoTimestamp(robotTimestamps[0] + 1);
        int insertionPointIndex = scrubber.getCurrentIndex();

        assertEquals(1, insertionPointIndex);
    }
}
