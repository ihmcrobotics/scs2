package us.ihmc.scs2.session;

import java.util.concurrent.TimeUnit;

import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class can be used to measure execution time of some code. The time variable is backed by a
 * {@link YoVariable} allowing to visualize the timer in the SCS2 GUI.
 */
public class YoTimer
{
   private final YoDouble timer;
   private final double scaleFromNanoseconds;

   private long start = -1;
   private long lastUpdateTime = -1;

   /**
    * Creates a new instance of a timer.
    * 
    * @param namePrefix the name prefix used for creating the time variable.
    * @param timerUnit  the unit to use for storing time measurements.
    * @param registry   the registry to attach the internal variable to.
    */
   public YoTimer(String namePrefix, TimeUnit timerUnit, YoRegistry registry)
   {
      timer = new YoDouble(namePrefix + "[" + timeUnitSuffix(timerUnit) + "]", registry);
      scaleFromNanoseconds = scaleFromNanoseconds(timerUnit);
   }

   private static String timeUnitSuffix(TimeUnit timeUnit)
   {
      switch (timeUnit)
      {
         case DAYS:
            return "day";
         case HOURS:
            return "hour";
         case MINUTES:
            return "min";
         case SECONDS:
            return "sec";
         case MILLISECONDS:
            return "ms";
         case MICROSECONDS:
            return "us";
         case NANOSECONDS:
            return "ns";
         default:
            throw new IllegalArgumentException("Unexpected value: " + timeUnit);
      }
   }

   private static double scaleFromNanoseconds(TimeUnit timeUnit)
   {
      switch (timeUnit)
      {
         case DAYS:
            return 1.0 / 86400.0 * 1.0e-9;
         case HOURS:
            return 1.0 / 3600.0 * 1.0e-9;
         case MINUTES:
            return 1.0 / 60.0 * 1.0e-9;
         case SECONDS:
            return 1.0e-9;
         case MILLISECONDS:
            return 1.0e-6;
         case MICROSECONDS:
            return 1.0e-3;
         case NANOSECONDS:
            return 1.0;
         default:
            throw new IllegalArgumentException("Unexpected value: " + timeUnit);
      }
   }

   /**
    * Marks the start of a new measurement.
    * <p>
    * Uses {@link System#nanoTime()} to get the current time.
    * </p>
    * <p>
    * The measurement will be the time elapsed from start to stop.
    * </p>
    */
   public void start()
   {
      start(System.nanoTime());
   }

   /**
    * Marks the start of a new measurement.
    * <p>
    * The measurement will be the time elapsed from start to stop.
    * </p>
    * 
    * @param now      the current time.
    * @param timeUnit the unit for {@code now}.
    */
   public void start(long now, TimeUnit timeUnit)
   {
      start(timeUnit.toNanos(now));
   }

   /**
    * Marks the start of a new measurement.
    * <p>
    * The measurement will be the time elapsed from start to stop.
    * </p>
    * 
    * @param nowNanosec the current time in nanoseconds.
    */
   public void start(long nowNanosec)
   {
      start = nowNanosec;
   }

   /**
    * Marks the end of the current measurement and update the timer variable. The measurement is the
    * time elapsed since start was called.
    * <p>
    * Uses {@link System#nanoTime()} to get the current time.
    * </p>
    * 
    * @throws IllegalOperationException if {@link #start} was not called last.
    */
   public void stop()
   {
      stop(System.nanoTime());
   }

   /**
    * Marks the end of the current measurement and update the timer variable. The measurement is the
    * time elapsed since start was called.
    * 
    * @param now      the current time.
    * @param timeUnit the unit for {@code now}.
    * @throws IllegalOperationException if {@link #start} was not called last.
    */
   public void stop(long now, TimeUnit timeUnit)
   {
      stop(timeUnit.toNanos(now));
   }

   /**
    * Marks the end of the current measurement and update the timer variable. The measurement is the
    * time elapsed since start was called.
    * 
    * @param nowNanosec the current time in nanoseconds.
    * @throws IllegalOperationException if {@link #start} was not called last.
    */
   public void stop(long nowNanosec)
   {
      if (start == -1)
         throw new IllegalOperationException("Call start before each call of stop.");

      timer.set((nowNanosec - start) * scaleFromNanoseconds);
      start = -1;
   }

   /**
    * Clears the internal data but keeps the timer value.
    */
   public void reset()
   {
      start = -1;
      lastUpdateTime = -1;
   }

   /**
    * Updates the time measurement as the time elapsed since the last call to update.
    * <p>
    * Uses {@link System#nanoTime()} to get the current time.
    * </p>
    */
   public void update()
   {
      update(System.nanoTime());
   }

   /**
    * Updates the time measurement as the time elapsed since the last call to update.
    * 
    * @param now      the current time.
    * @param timeUnit the unit for {@code now}.
    */
   public void update(long now, TimeUnit timeUnit)
   {
      update(timeUnit.toNanos(now));
   }

   /**
    * Updates the time measurement as the time elapsed since the last call to update.
    * 
    * @param nowNanosec the current time in nanoseconds.
    */
   public void update(long nowNanosec)
   {
      if (lastUpdateTime > -1)
      {
         timer.set((double) (nowNanosec - lastUpdateTime) * scaleFromNanoseconds);
      }

      lastUpdateTime = nowNanosec;
   }

   /**
    * Gets the timer variable that holds onto the last time measurement.
    * 
    * @return the timer variable.
    */
   public YoDouble getTimer()
   {
      return timer;
   }
}
