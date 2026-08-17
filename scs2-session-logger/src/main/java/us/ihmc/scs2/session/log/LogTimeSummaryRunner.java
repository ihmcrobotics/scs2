package us.ihmc.scs2.session.log;

import java.io.File;
import java.io.IOException;
import java.util.List;

import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Command-line runner that opens a robot data log and reports the tick rate of the thread(s)
 * behind one or more chosen logged variables, instead of naively dividing the log's total tick
 * count by a variable's own first/last value.
 * <p>
 * Every log ticks is written at whatever cadence the logger's fastest thread runs at. A slower
 * thread's own variables don't change on every one of those ticks - they sit at the same value
 * for several ticks until that thread ticks again. So {@code numberOfEntries} (the raw tick
 * count) is <b>not</b> the number of real ticks of an arbitrary variable's producing thread, and
 * the variable's own value can't be trusted as a wall clock either. This instead:
 * <ul>
 * <li>uses each tick's own logged timestamp (nanoseconds, independent of any particular
 * thread) for wall-clock duration, and
 * <li>counts real ticks of the chosen variable rather than raw records - exactly via a
 * first/last counter difference if it's a {@code LongYoVariable} tick counter, or by scanning
 * the log and counting value changes if it's a {@code DoubleYoVariable} clock with no counter
 * available.
 * </ul>
 * The counter mode only reads the first and last record and returns almost immediately. The
 * change-detection mode has to stream the whole log to find every transition, so it's slower;
 * prefer pointing this at a {@code LongYoVariable} tick counter for that thread when one exists.
 * By default this runs against the tick counters of the scheduler, estimator, and controller
 * threads so their rates can be compared side by side in one pass.
 * <p>
 * Usage: {@code LogTimeSummaryRunner [timeVariableName ...]}
 * <ul>
 * <li>{@code timeVariableName} — a substring of a logged time variable's name; one or more may be
 * given, each reported separately. Defaults to {@link #DEFAULT_TIME_VARIABLE_NAMES}. The first
 * logged variable whose name contains it is used.
 * </ul>
 */
public class LogTimeSummaryRunner
{
    private static final String[] DEFAULT_TIME_VARIABLE_NAMES = {"SchedulerTick", "EstimatorTick", "ControllerTick"};
    // This is where the user can set the log path for debugging the threading rates
    private static final String LOG_PATH = "/opt/ihmc/LogData/incoming/20260814_084930_Alex002UnifiedControlProcess";

    public static void main(String[] args) throws IOException
    {
        // Locate the log directory and bail out early if it's missing or empty.
        File logDirectory = new File(LOG_PATH);
        if (!logDirectory.isDirectory())
        {
            System.err.println("Not a directory: " + logDirectory.getAbsolutePath());
            System.exit(1);
            return;
        }

        String[] timeVariableNames = args.length > 0 ? args : DEFAULT_TIME_VARIABLE_NAMES;

        // Opens the handshake + index so we know how many records exist and can seek into the data file.
        LogDataReader logDataReader = new LogDataReader(logDirectory, new SilentProgressConsumer());

        int numberOfEntries = logDataReader.getNumberOfEntries();
        if (numberOfEntries <= 0)
        {
            System.err.println("Log contains no entries.");
            System.exit(1);
            return;
        }

        System.out.println("Log directory   : " + logDirectory.getAbsolutePath());
        System.out.println("Log entries     : " + numberOfEntries + " (raw ticks; not the same as real ticks of any one thread)");

        // Each variable is reported independently, using the same LogDataReader; every report
        // method re-seeks from scratch so runs don't interfere with each other.
        for (String timeVariableName : timeVariableNames)
        {
            System.out.println();

            YoVariable timeVariable = findVariable(logDataReader, timeVariableName);
            if (timeVariable == null)
            {
                System.err.println("Could not find a logged variable whose name contains \"" + timeVariableName + "\".");
                continue;
            }

            System.out.println("Time variable   : " + timeVariable.getFullNameString());

            // Long tick counters give an exact answer from just two reads; anything else needs a full scan
            // to detect real ticks (see the two report* methods below for why).
            if (timeVariable instanceof YoLong)
                reportCounterMode(logDataReader, (YoLong) timeVariable, numberOfEntries);
            else
                reportChangeDetectionMode(logDataReader, timeVariable, numberOfEntries);
        }
    }

    /**
     * Exact mode for a {@code LongYoVariable} tick counter: the difference between its first and
     * last value over the wall-clock span of those two records is the real tick count, no
     * scanning required.
     */
    private static void reportCounterMode(LogDataReader logDataReader, YoLong counterVariable, int numberOfEntries)
    {
        // logDataReader.getTimestamp() is the per-record wall-clock timestamp written by the logger
        // itself, independent of any thread's own variables - that's what we use as the clock.
        YoLong timestamp = logDataReader.getTimestamp();

        logDataReader.seek(0);
        logDataReader.read();
        long firstTimestamp = timestamp.getLongValue();
        long firstCount = counterVariable.getLongValue();

        logDataReader.seek(numberOfEntries - 1);
        logDataReader.read();
        long lastTimestamp = timestamp.getLongValue();
        long lastCount = counterVariable.getLongValue();

        // A tick counter only increments once per real tick, so this difference is exact.
        long realTicks = lastCount - firstCount;
        double duration = (lastTimestamp - firstTimestamp) / 1.0e9;
        double frequency = duration > 0 ? realTicks / duration : Double.NaN;

        System.out.println("Mode            : counter (exact difference)");
        System.out.println("Real ticks      : " + realTicks);
        System.out.println("Duration        : " + duration + " s (from record timestamps)");
        System.out.println("Frequency       : " + frequency + " Hz");
    }

    /**
     * Fallback for a {@code DoubleYoVariable} clock with no tick counter available: stream the
     * whole log and count how many times the value actually changes. Duration is measured
     * between the first record and the record of the last observed change, using each record's
     * own timestamp - not the variable's value - as the wall clock.
     */
    private static void reportChangeDetectionMode(LogDataReader logDataReader, YoVariable timeVariable, int numberOfEntries)
    {
        YoLong timestamp = logDataReader.getTimestamp();

        logDataReader.seek(0);
        logDataReader.read();
        long firstTimestamp = timestamp.getLongValue();
        double firstValue = timeVariable.getValueAsDouble();

        double previousValue = firstValue;
        double lastValue = firstValue;
        long lastChangeTimestamp = firstTimestamp;
        long realTicks = 0;

        // Walk every record in order; a real tick is a record where the value differs from the
        // previous one. Records where the producing thread hasn't run again yet just repeat the
        // same value and don't count.
        for (int position = 1; position < numberOfEntries; position++)
        {
            logDataReader.read();
            double currentValue = timeVariable.getValueAsDouble();
            if (currentValue != previousValue)
            {
                realTicks++;
                lastChangeTimestamp = timestamp.getLongValue();
                lastValue = currentValue;
                previousValue = currentValue;
            }
        }

        double duration = (lastChangeTimestamp - firstTimestamp) / 1.0e9;
        double frequency = duration > 0 ? realTicks / duration : Double.NaN;

        System.out.println("Mode            : clock (change-detection, scanned full log)");
        System.out.println("Real ticks      : " + realTicks + " (times the value actually changed)");
        System.out.println("First value     : " + firstValue);
        System.out.println("Last value      : " + lastValue);
        System.out.println("Duration        : " + duration + " s (from record timestamps, first record to last observed change)");
        System.out.println("Frequency       : " + frequency + " Hz");
    }

    /** Returns the first logged variable whose simple name contains {@code variableName}, or {@code null}. */
    private static YoVariable findVariable(LogDataReader logDataReader, String variableName)
    {
        List<YoVariable> yoVariables = logDataReader.getYoVariablesList();

        YoVariable firstMatch = null;
        int matchCount = 0;
        for (YoVariable yoVariable : yoVariables)
        {
            if (yoVariable.getName().contains(variableName))
            {
                if (firstMatch == null)
                    firstMatch = yoVariable;
                matchCount++;
            }
        }

        if (matchCount > 1)
            System.out.println(matchCount + " variables contain \"" + variableName + "\"; using " + firstMatch.getFullNameString()
                    + ". Pass a more specific name to disambiguate.");

        return firstMatch;
    }

    /** No-op progress consumer so opening the log stays quiet on the console. */
    private static class SilentProgressConsumer implements ProgressConsumer
    {
        @Override
        public void started(String task)
        {
        }

        @Override
        public void info(String info)
        {
        }

        @Override
        public void error(String error)
        {
            System.err.println(error);
        }

        @Override
        public void progress(double progressPercentage)
        {
        }

        @Override
        public void done()
        {
        }
    }
}
