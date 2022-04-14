package us.ihmc.scs2.session;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class gather generic properties of a ongoing session.
 */
public class SessionProperties
{
   /**
    * The current mode the session is running, see {@link SessionMode}.
    */
   private final SessionMode activeMode;
   /**
    * Whether the {@link SessionMode#RUNNING} mode should be capped to run no faster that real-time.
    */
   private final boolean runAtRealTimeRate;
   /** The speed at which the {@link SessionMode#PLAYBACK} should play back the buffered data. */
   private final double playbackRealTimeRate;
   /**
    * Maps from one session run tick to the time increment in the data:
    * <ul>
    * <li>when simulating, this corresponds to the simulation DT,
    * <li>when reading a log, this corresponds to the period at which data was stored,
    * <li>when working with a remote session, this corresponds to the period at which the server is
    * streaming data.
    * </ul>
    */
   private final long sessionDTNanoseconds;
   /**
    * The number of times {@link #runTick()} should be called before saving the {@link YoVariable} data
    * into the buffer.
    * <p>
    * A larger value allows to store data over longer period of time.
    * </p>
    */
   private final int bufferRecordTickPeriod;

   public SessionProperties(SessionMode activeMode,
                            boolean runAtRealTimeRate,
                            double playbackRealTimeRate,
                            long sessionDTNanoseconds,
                            int bufferRecordTickPeriod)
   {
      this.activeMode = activeMode;
      this.runAtRealTimeRate = runAtRealTimeRate;
      this.playbackRealTimeRate = playbackRealTimeRate;
      this.sessionDTNanoseconds = sessionDTNanoseconds;
      this.bufferRecordTickPeriod = bufferRecordTickPeriod;
   }

   /**
    * Gets the current mode the session is running, see {@link SessionMode}.
    * 
    * @return the active mode.
    * @see SessionMode
    */
   public SessionMode getActiveMode()
   {
      return activeMode;
   }

   /**
    * Whether the {@link SessionMode#RUNNING} mode should be capped to run no faster that real-time.
    * 
    * @return {@code true} if the running mode is capped to real-time rate, {@code false} if it is
    *         executes as frequently as possible.
    */
   public boolean isRunAtRealTimeRate()
   {
      return runAtRealTimeRate;
   }

   /**
    * The speed at which the {@link SessionMode#PLAYBACK} should play back the buffered data.
    * 
    * @return the desired real-time factor for playback speed.
    */
   public double getPlaybackRealTimeRate()
   {
      return playbackRealTimeRate;
   }

   /**
    * Gets the time increment from one session run tick:
    * <ul>
    * <li>when simulating, this corresponds to the simulation DT,
    * <li>when reading a log, this corresponds to the period at which data was stored,
    * <li>when working with a remote session, this corresponds to the period at which the server is
    * streaming data.
    * </ul>
    * 
    * @return the session DT in nanoseconds.
    */
   public long getSessionDTNanoseconds()
   {
      return sessionDTNanoseconds;
   }

   /**
    * Gets the number of times {@link #runTick()} should be called before saving the {@link YoVariable}
    * data into the buffer.
    * <p>
    * A larger value allows to store data over longer period of time.
    * </p>
    * 
    * @return the period, in number of run ticks, that the data is stored in the buffer.
    */
   public int getBufferRecordTickPeriod()
   {
      return bufferRecordTickPeriod;
   }
}
