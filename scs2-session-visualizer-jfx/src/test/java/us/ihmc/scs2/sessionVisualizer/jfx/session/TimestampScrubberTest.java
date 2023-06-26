package us.ihmc.scs2.sessionVisualizer.jfx.session;

import org.apache.commons.math.stat.descriptive.moment.StandardDeviation;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.VideoDataReader;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests in this class are disabled because the files have duplicate timestamps that will cause issues when trying to retrieve a specific timestamp
 */

public class TimestampScrubberTest
{
    private VideoDataReader.TimestampScrubber scrubber;

    private long[] robotTimestamps;
    private long[] videoTimestamps;

    @BeforeEach
    public void loadFileTimestamps() throws URISyntaxException, IOException
    {
        File timestampFile = new File(Objects.requireNonNull(getClass().getClassLoader().getResource("sessionLogs/Capture.dat")).toURI());

        scrubber = new VideoDataReader.TimestampScrubber(timestampFile, true, false);

        // Need to have one video in the log or this will fail
        robotTimestamps = scrubber.getRobotTimestampsArray();
        videoTimestamps = scrubber.getVideoTimestampsArray();
    }

    @Test
    public void testChronologicallyIncreasingRobotTimestamps()
    {
        long previousTimestamp;

        // Go through the robot timestamps in order and check the next one is larger
        for (int i = 1; i < robotTimestamps.length; i++)
        {
            previousTimestamp = robotTimestamps[i - 1];
            long currentTimestamp = robotTimestamps[i];

            //TODO fix duplicate timestamps
            if (currentTimestamp == previousTimestamp)
                continue;

            Assertions.assertTrue(currentTimestamp > previousTimestamp, "Cureent: " + currentTimestamp + "\n Previous: " + previousTimestamp);
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

    @Disabled
    @Test
    public void testDeltaStatisticsBetweenRobotTimestamps()
    {
        long previousTimestamp;
        long delta = 0;
        long smallestDelta = 1000000000;
        long largestDelta = 0;
        int duplicates = 0;

        // Go through the robot timestamps in order and check the next one is larger
        for (int i = 1; i < robotTimestamps.length; i++)
        {
            previousTimestamp = robotTimestamps[i - 1];
            long currentTimestamp = robotTimestamps[i];

            Assertions.assertTrue(currentTimestamp > previousTimestamp, "Cureent: " + currentTimestamp + "\n Previous: " + previousTimestamp);

            long currentDelta = currentTimestamp - previousTimestamp;
            delta += currentDelta;

            if (currentDelta < smallestDelta)
                smallestDelta = currentDelta;

            if (currentDelta > largestDelta)
                largestDelta = currentDelta;
        }

        delta = delta / (robotTimestamps.length - duplicates);

        System.out.println("Smallest Delta: " + smallestDelta);
        System.out.println("Largest Delta: " + largestDelta);
        System.out.println("Duplicate robotTimestamps: " + duplicates);
        System.out.println("Average delta for robotTimestamps: " + delta);
    }

    @Disabled
    @Test
    public void testGoingThroughRobotTimestampsInOrder()
    {
        // Go through the robot timestamps in order and see if we get the desired video timestamp
        for (int i = 0; i < robotTimestamps.length; i++)
        {
            scrubber.getVideoTimestamp(robotTimestamps[i]);
            assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[i]);
        }
    }

    @Disabled
    @Test
    public void testGoingThroughRobotTimestampsEveryOther()
    {
        // Go through the robot timestamps by +=2, so we skip every other frame and see if we get the desired video timestamp
        for (int i = 0; i < robotTimestamps.length ; i+=2)
        {
            scrubber.getVideoTimestamp(robotTimestamps[i]);
            assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[i], "For look index: " + i);
        }
    }

    @Disabled
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

    @Disabled
    @Test
    public void testGoingThroughRobotTimestampsBackwards()
    {
        for (int i = robotTimestamps.length - 1; i > 0; i--)
        {
            scrubber.getVideoTimestamp(robotTimestamps[i]);
            assertEquals(scrubber.getCurrentVideoTimestamp(), videoTimestamps[i]);
        }
    }

    // Method to prevent reading duplicate timestamps, useful for testing future tests and bugs
    public boolean robotTimestampIsNotUnique(int index)
    {
        boolean checkPrevious = false;
        boolean checkNext = false;

        if (index - 1 > 0)
            checkPrevious = robotTimestamps[index] == robotTimestamps[index - 1];

        if (index + 1 < robotTimestamps.length)
            checkNext = robotTimestamps[index] == robotTimestamps[index + 1];

        return checkPrevious || checkNext;
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
