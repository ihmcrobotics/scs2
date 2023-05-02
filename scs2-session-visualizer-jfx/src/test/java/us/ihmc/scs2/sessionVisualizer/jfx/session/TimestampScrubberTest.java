package us.ihmc.scs2.sessionVisualizer.jfx.session;

import org.apache.commons.math.stat.descriptive.moment.StandardDeviation;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.VideoDataReader;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.Objects;

public class TimestampScrubberTest
{
    private VideoDataReader.TimestampScrubber scrubber;

    private long[] actualRobotTimestamps;
    private long[] actualVideoTimestamps;

    @BeforeEach
    public void loadFileTimestamps() throws URISyntaxException, IOException
    {
        File timestampFile = new File(Objects.requireNonNull(getClass().getClassLoader().getResource("sessionLogs/NadiaPoleNorth_Timestamps.dat")).toURI());

        scrubber = new VideoDataReader.TimestampScrubber(timestampFile, true, false);

        // Need to have one video in the log or this will fail
        actualRobotTimestamps = scrubber.getRobotTimestampsFromFile();
        actualVideoTimestamps = scrubber.getVideoTimestampsFromFile();
    }

    @Test
    public void testChronologicallyIncreasingRobotTimestamps()
    {
        long previousTimestamp;

        // Go through the robot timestamps in order and check the next one is larger
        for (int i = 1; i < actualRobotTimestamps.length; i++)
        {
            previousTimestamp = actualRobotTimestamps[i - 1];
            long currentTimestamp = actualRobotTimestamps[i];

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

        // Go through the robot timestamps in order and check the next one is larger
        for (int i = 1; i < actualRobotTimestamps.length; i++)
        {
            previousTimestamp = actualRobotTimestamps[i - 1];
            long currentTimestamp = actualRobotTimestamps[i];

            //TODO fix duplicate timestamps
            if (currentTimestamp == previousTimestamp)
            {
                System.out.println("Current:" + currentTimestamp + " and Previous: " + previousTimestamp);
                duplicates++;
                continue;
            }

            Assertions.assertTrue(currentTimestamp > previousTimestamp, "Cureent: " + currentTimestamp + "\n Previous: " + previousTimestamp);

            long currentDelta = currentTimestamp - previousTimestamp;
            delta += currentDelta;

            if (currentDelta < smallestDelta)
                smallestDelta = currentDelta;

            if (currentDelta > largestDelta)
                largestDelta = currentDelta;
        }

        delta = delta / (actualRobotTimestamps.length - duplicates);

        System.out.println("Smallest Delta: " + smallestDelta);
        System.out.println("Largest Delta: " + largestDelta);
        System.out.println("Duplicate robotTimestamps: " + duplicates);
        System.out.println("Average delta for robotTimestamps: " + delta);
    }

    @Test
    public void testGoingThroughRobotTimestampsInOrder()
    {
        // Go through the robot timestamps in order and see if we get the desired video timestamp
        for (int i = 0; i < actualRobotTimestamps.length; i++)
        {
            //TODO there really shouldn't be duplicate robotTimestamps so that seems like an issue

            // Need unique robot timestamp, otherwise how could we possibly find the unique video timestamp
            if (robotTimestampIsNotUnique(i))
            {
                System.out.println(actualRobotTimestamps[i]);
                continue;
            }
            scrubber.setCorrectVideoTimestamp(actualRobotTimestamps[i]);
            Assertions.assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[i]);
        }
    }

    @Test
    public void testGoingThroughRobotTimestampsEveryOther()
    {
        // Go through the robot timestamps by +=2, so we skip every other frame and see if we get the desired video timestamp
        for (int i = 0; i < actualRobotTimestamps.length ; i+=2)
        {
            // Need unique robot timestamp, otherwise how could we possibly find the unique video timestamp
            if (robotTimestampIsNotUnique(i))
                continue;

            scrubber.setCorrectVideoTimestamp(actualRobotTimestamps[i]);
            Assertions.assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[i], "For look index: " + i);
        }
    }

    @Test
    public void testGettingRandomTimestamp()
    {
        // Test grabbing random robot timestamps and checking to make sure we get the correct video timestamp
        // These robot timestamps need to be unique or the binary search will fail to get the correct video timestamp
        scrubber.setCorrectVideoTimestamp(actualRobotTimestamps[26]);
        Assertions.assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[26]);

        scrubber.setCorrectVideoTimestamp(actualRobotTimestamps[40]);
        Assertions.assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[40]);

        scrubber.setCorrectVideoTimestamp(actualRobotTimestamps[34]);
        Assertions.assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[34]);
    }

    @Test
    public void testGoingThroughRobotTimestampsBackwards()
    {
        for (int i = actualRobotTimestamps.length - 1; i > 0; i--)
        {
            // Need unique robot timestamp, otherwise how could we possibly find the unique video timestamp
            if (robotTimestampIsNotUnique(i))
                continue;

            scrubber.setCorrectVideoTimestamp(actualRobotTimestamps[i]);
            Assertions.assertEquals(scrubber.getCurrentVideoTimestamp(), actualVideoTimestamps[i]);
        }
    }

    public boolean robotTimestampIsNotUnique(int index)
    {
        boolean checkPrevious = false;
        boolean checkNext = false;

        if (index - 1 > 0)
            checkPrevious = actualRobotTimestamps[index] == actualRobotTimestamps[index - 1];

        if (index + 1 < actualRobotTimestamps.length)
            checkNext = actualRobotTimestamps[index] == actualRobotTimestamps[index + 1];

        return checkPrevious || checkNext;
    }
}
