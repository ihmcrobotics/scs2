package us.ihmc.scs2.session;

import java.io.IOException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import javax.xml.bind.JAXBException;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.TopicListener;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.FillBufferRequest;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Base class for implementing a session, e.g. a simulation, log reading, or a remote session.
 */
public abstract class Session
{
   private static final int DEFAULT_INITIAL_BUFFER_SIZE = SessionPropertiesHelper.loadIntegerProperty("scs2.session.buffer.initialsize", 8192);
   private static final int DEFAULT_BUFFER_RECORD_TICK_PERIOD = SessionPropertiesHelper.loadIntegerProperty("scs2.session.buffer.recordtickperiod", 1);
   private static final boolean DEFAULT_RUN_AT_REALTIME_RATE = SessionPropertiesHelper.loadBooleanProperty("scs2.session.runrealtime", false);
   private static final double DEFAULT_PLAYBACK_REALTIME_RATE = SessionPropertiesHelper.loadDoubleProperty("scs2.session.playrealtime", 1.0);
   private static final long DEFAULT_BUFFER_PUBLISH_PERIOD = SessionPropertiesHelper.loadLongProperty("scs2.session.buffer.publishperiod",
                                                                                                      (long) (1.0 / 30.0 * 1.0e9));

   /** Name of the root registry for any session. */
   public static final String ROOT_REGISTRY_NAME = "root";
   /** Name of the registry that will contains variables related to the internal state of SCS2. */
   public static final String SESSION_INTERNAL_REGISTRY_NAME = Session.class.getSimpleName() + "InternalRegistry";
   /** Namespace of the root registry for any session. */
   public static final YoNamespace ROOT_NAMESPACE = new YoNamespace(ROOT_REGISTRY_NAME);
   /** Namespace of the registry that will contains variables related to the internal state of SCS2. */
   public static final YoNamespace SESSION_INTERNAL_NAMESPACE = ROOT_NAMESPACE.append(SESSION_INTERNAL_REGISTRY_NAME);
   /**
    * Name suffix for any {@link ReferenceFrame} that only serve internal purpose as for instance
    * helping with the physics engine's calculation.
    */
   public static final String SCS2_INTERNAL_FRAME_SUFFIX = "[SCS2Internal]";

   /**
    * The inertial frame associated to this session. It is expected to be a root frame that is
    * different to {@link ReferenceFrame#getWorldFrame()}.
    */
   protected final ReferenceFrame inertialFrame;

   /**
    * The instance of the root registry of this session. Every {@link YoRegistry} and
    * {@link YoVariable} that are to be buffered must be attached directly or indirectly to the root
    * registry.
    */
   protected final YoRegistry rootRegistry = new YoRegistry(ROOT_REGISTRY_NAME);
   /**
    * The instance of the registry that is used to register variables related to the internal state of
    * SCS2.
    */
   protected final YoRegistry sessionRegistry = new YoRegistry(SESSION_INTERNAL_REGISTRY_NAME);
   /**
    * Variable holding the current time (in seconds) for this session. It represents notably:
    * <ul>
    * <li>the amount of simulation time when working with a simulation session;
    * <li>the amount of time since the start of a log file when working with a log session;
    * <li>the time broadcasted by the server when working with a remote session.
    * </ul>
    */
   protected final YoDouble time = new YoDouble("time[sec]", rootRegistry);
   /**
    * JVM statistics for this session, allowing for instance to inspect garbage collection or other
    * indicator for performance debugging.
    */
   private final JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator("SCS2Stats", sessionRegistry);

   /** Registry gathering debug variables related to {@link #runTick()}. */
   protected final YoRegistry runRegistry = new YoRegistry("runStatistics");
   /** Timer used to measured the time elapsed between 2 calls of {@link #runTick()}. */
   private final YoTimer runActualDT = new YoTimer("runActualDT", TimeUnit.MILLISECONDS, runRegistry);
   /** Timer used to measured the total time spent in each call of {@link #runTick()}. */
   private final YoTimer runTimer = new YoTimer("runTimer", TimeUnit.MILLISECONDS, runRegistry);
   /** Timer used to measured the total time spent in each call of {@link #initializeRunTick()}. */
   private final YoTimer runInitializeTimer = new YoTimer("runInitializeTimer", TimeUnit.MILLISECONDS, runRegistry);
   /** Timer used to measured the total time spent in each call of {@link #doSpecificRunTick()}. */
   private final YoTimer runSpecificTimer = new YoTimer("runSpecificTimer", TimeUnit.MILLISECONDS, runRegistry);
   /**
    * Timer used to measured the total time spent in each call of {@link #finalizeRunTick(boolean)}.
    */
   private final YoTimer runFinalizeTimer = new YoTimer("runFinalizeTimer", TimeUnit.MILLISECONDS, runRegistry);

   /** Registry gathering debug variables related to {@link #playbackTick()}. */
   protected final YoRegistry playbackRegistry = new YoRegistry("playbackStatistics");
   /** Timer used to measured the time elapsed between 2 calls of {@link #playbackTick()}. */
   private final YoTimer playbackActualDT = new YoTimer("playbackActualDT", TimeUnit.MILLISECONDS, playbackRegistry);
   /** Timer used to measured the total time spent in each call of {@link #playbackTick()}. */
   private final YoTimer playbackTimer = new YoTimer("playbackTimer", TimeUnit.MILLISECONDS, playbackRegistry);

   /** Registry gathering debug variables related to {@link #pauseTick()}. */
   protected final YoRegistry pauseRegistry = new YoRegistry("pauseStatistics");
   /** Timer used to measured the time elapsed between 2 calls of {@link #pauseTick()}. */
   private final YoTimer pauseActualDT = new YoTimer("pauseActualDT", TimeUnit.MILLISECONDS, pauseRegistry);
   /** Timer used to measured the total time spent in each call of {@link #pauseTick()}. */
   private final YoTimer pauseTimer = new YoTimer("pauseTimer", TimeUnit.MILLISECONDS, pauseRegistry);

   /**
    * Instance of the buffer used for this session. It is used to keep track of the history of every
    * {@link YoVariable} registered as a descendant of the {@link #rootRegistry}.
    */
   protected final YoSharedBuffer sharedBuffer = new YoSharedBuffer(rootRegistry, DEFAULT_INITIAL_BUFFER_SIZE);
   /**
    * The current mode this session is running, see {@link SessionMode}.
    */
   private final AtomicReference<SessionMode> activeMode = new AtomicReference<>(SessionMode.PAUSE);
   /**
    * Whether the {@link SessionMode#RUNNING} mode should be capped to run no faster that real-time.
    */
   private final AtomicBoolean runAtRealTimeRate = new AtomicBoolean(DEFAULT_RUN_AT_REALTIME_RATE);
   /** The speed at which the {@link SessionMode#PLAYBACK} should play back the buffered data. */
   private final AtomicReference<Double> playbackRealTimeRate = new AtomicReference<>(DEFAULT_PLAYBACK_REALTIME_RATE);
   /**
    * The number of ticks to step while in playback mode. Allows to play back at faster rates by
    * increasing the step size.
    */
   private int stepSizePerPlaybackTick = 1;
   /**
    * The number of times {@link #runTick()} should be called before saving the {@link YoVariable} data
    * into the buffer.
    * <p>
    * A larger value allows to store data over longer period of time.
    * </p>
    */
   private final AtomicInteger bufferRecordTickPeriod = new AtomicInteger(DEFAULT_BUFFER_RECORD_TICK_PERIOD);
   /**
    * Map from one session run tick to the time increment in the data.
    * <p>
    * When simulating, this corresponds to the simulation DT for instance.
    * </p>
    */
   private final AtomicLong sessionDTNanoseconds = new AtomicLong(Conversions.secondsToNanoseconds(1.0e-4));
   /**
    * Period for controlling the rate at which the GUI will be refreshed.
    * <p>
    * This is particularly useful for instance when reading a log, increasing this period allows to
    * increase the speed at which we can read the log, also useful for the remote session to reduce the
    * computational induced by the GUI while receiving a high-bandwidth stream of data.
    * </p>
    */
   private final AtomicLong desiredBufferPublishPeriod = new AtomicLong(DEFAULT_BUFFER_PUBLISH_PERIOD);

   // State listener to publish internal to outside world
   /** Period at which the current session properties are to be published. */
   private final long sessionPropertiesPublishPeriod = 500L;
   /** To keep track of the last time the session properties were published. */
   private long lastSessionPropertiesPublishTimestamp = -1L;
   // Listeners
   /** Listeners that get notified right after the session mode has changed. */
   private final List<SessionModeChangeListener> sessionModeChangeListeners = new ArrayList<>();
   /** Listeners that get notified right before the session mode has changed. */
   private final List<SessionModeChangeListener> preSessionModeChangeListeners = new ArrayList<>();
   private final List<Consumer<SessionProperties>> sessionPropertiesListeners = new ArrayList<>();
   private final List<Consumer<YoBufferPropertiesReadOnly>> currentBufferPropertiesListeners = new ArrayList<>();
   private final List<Runnable> shutdownListeners = new ArrayList<>();

   // For exception handling
   private final List<Consumer<Throwable>> runThrowableListeners = new ArrayList<>();
   private final List<Consumer<Throwable>> playbackThrowableListeners = new ArrayList<>();

   // Fields for external requests on buffer.
   private final SessionUserField<CropBufferRequest> pendingCropBufferRequest = new SessionUserField<>();
   private final SessionUserField<FillBufferRequest> pendingFillBufferRequest = new SessionUserField<>();
   private final SessionUserField<Integer> pendingBufferIndexRequest = new SessionUserField<>();
   private final SessionUserField<Integer> pendingBufferInPointIndexRequest = new SessionUserField<>();
   private final SessionUserField<Integer> pendingBufferOutPointIndexRequest = new SessionUserField<>();
   private final SessionUserField<Integer> pendingIncrementBufferIndexRequest = new SessionUserField<>();
   private final SessionUserField<Integer> pendingDecrementBufferIndexRequest = new SessionUserField<>();
   private final SessionUserField<Integer> pendingBufferSizeRequest = new SessionUserField<>();
   private final SessionUserField<SessionDataExportRequest> pendingDataExportRequest = new SessionUserField<>();

   // Strictly internal fields
   private final List<SessionTopicListenerManager> sessionTopicListenerManagers = new ArrayList<>();
   private boolean sessionThreadStarted = false;
   private boolean sessionInitialized = false;
   private boolean isSessionShutdown = false;
   private long lastPublishedBufferTimestamp = -1L;
   protected boolean firstRunTick = true;
   protected boolean firstPauseTick = true;

   protected final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(2, new DaemonThreadFactory("SCS2-Session-Thread"));
   protected PeriodicTaskWrapper activePeriodicTask;

   private final EnumMap<SessionMode, Runnable> sessionModeToTaskMap = new EnumMap<>(SessionMode.class);

   protected boolean hasBufferSizeBeenInitialized = false;
   protected boolean hasBufferRecordPeriodBeenInitialized = false;

   public Session()
   {
      this(ReferenceFrameTools.constructARootFrame("worldFrame"));
   }

   public Session(ReferenceFrame inertialFrame)
   {
      this.inertialFrame = inertialFrame;

      rootRegistry.addChild(sessionRegistry);
      sessionRegistry.addChild(runRegistry);
      sessionRegistry.addChild(playbackRegistry);
      sessionRegistry.addChild(pauseRegistry);

      setSessionModeTask(SessionMode.RUNNING, this::runTick);
      setSessionModeTask(SessionMode.PLAYBACK, this::playbackTick);
      setSessionModeTask(SessionMode.PAUSE, this::pauseTick);
   }

   /**
    * Attempts to retrieve the simple name of the main class calling this method.
    * 
    * @return the simple name of the main calling this method.
    */
   public static String retrieveCallerName()
   {
      StackTraceElement[] stackTrace = new Throwable().getStackTrace();
      String className = stackTrace[stackTrace.length - 1].getClassName();
      return className.substring(className.lastIndexOf(".") + 1);
   }

   /**
    * Configures this session to communicate with the given messager. The session will register
    * listeners and publishers using the API defined in {@link YoSharedBufferMessagerAPI} and
    * {@link SessionMessagerAPI}.
    * 
    * @param messager the messager to configure this session with.
    */
   public void setupWithMessager(Messager messager)
   {
      for (SessionTopicListenerManager manager : sessionTopicListenerManagers)
      {
         if (messager == manager.messager)
            throw new IllegalArgumentException("Messager already registered.");
      }

      sessionTopicListenerManagers.add(new SessionTopicListenerManager(messager));
   }

   /**
    * Requests the change to a new session mode.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param sessionMode the target mode.
    */
   public void setSessionMode(SessionMode sessionMode)
   {
      setSessionMode(sessionMode, null);
   }

   /**
    * Changes the current session mode and schedule a transition that will be used to determine when
    * the newly triggered mode is done, and when it is done what is the next mode to run.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param sessionMode the new active session mode.
    * @param transition  the transition to schedule termination of the given {@code sessionMode} and
    *                    transition to the following mode.
    */
   protected void setSessionMode(SessionMode sessionMode, SessionModeTransition transition)
   {
      SessionMode currentMode = activeMode.get();

      if (sessionMode != currentMode)
         scheduleSessionTask(sessionMode, transition);
   }

   /**
    * Adds a listener to be notified whenever this session just changed mode.
    * 
    * @param listener the listener to add.
    * @see SessionMode
    */
   public void addSessionModeChangeListener(SessionModeChangeListener listener)
   {
      sessionModeChangeListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removeSessionModeChangeListener(SessionModeChangeListener listener)
   {
      return sessionModeChangeListeners.remove(listener);
   }

   /**
    * Adds a listener to be notified whenever this session is about to change mode.
    * 
    * @param listener the listener to add.
    * @see SessionMode
    */
   public void addPreSessionModeChangeListener(SessionModeChangeListener listener)
   {
      preSessionModeChangeListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removePreSessionModeChangeListener(SessionModeChangeListener listener)
   {
      return preSessionModeChangeListeners.remove(listener);
   }

   /**
    * Adds a listener to be notified whenever this session is about to shutdown.
    * 
    * @param listener the listener to add.
    */
   public void addShutdownListener(Runnable listener)
   {
      shutdownListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removeShutdownListener(Runnable listener)
   {
      return shutdownListeners.remove(listener);
   }

   /**
    * Adds a listener to be notified on a regular basis about this session's general parameters.
    * 
    * @param listener the listener to add.
    * @see SessionProperties
    */
   public void addSessionPropertiesListener(Consumer<SessionProperties> listener)
   {
      sessionPropertiesListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removeSessionPropertiesListener(Consumer<SessionProperties> listener)
   {
      return sessionPropertiesListeners.remove(listener);
   }

   /**
    * Adds a listener to be notified on a regular basis about this session's buffer properties.
    * 
    * @param listener the listener to add.
    * @see YoBufferPropertiesReadOnly
    */
   public void addCurrentBufferPropertiesListener(Consumer<YoBufferPropertiesReadOnly> listener)
   {
      currentBufferPropertiesListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removeCurrentBufferPropertiesListener(Consumer<YoBufferPropertiesReadOnly> listener)
   {
      return currentBufferPropertiesListeners.remove(listener);
   }

   /**
    * Adds a listener to be notified if an exception is being thrown while this session is in running
    * mode
    * 
    * @param listener the listener to add.
    * @see SessionMode
    */
   public void addRunThrowableListener(Consumer<Throwable> listener)
   {
      runThrowableListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removeRunThrowableListener(Consumer<Throwable> listener)
   {
      return runThrowableListeners.remove(listener);
   }

   /**
    * Adds a listener to be notified if an exception is being thrown while this session is in playback
    * mode
    * 
    * @param listener the listener to add.
    * @see SessionMode
    */
   public void addPlaybackThrowableListener(Consumer<Throwable> listener)
   {
      playbackThrowableListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removePlaybackThrowableListener(Consumer<Throwable> listener)
   {
      return playbackThrowableListeners.remove(listener);
   }

   /**
    * Sets the DT, i.e. time increment in seconds, corresponding to one tick of this session when in
    * running mode.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param sessionDTSeconds the time increment in seconds per running tick.
    * @see SessionMode
    */
   public void setSessionDTSeconds(double sessionDTSeconds)
   {
      setSessionDTNanoseconds(Conversions.secondsToNanoseconds(sessionDTSeconds));
   }

   /**
    * Sets the DT, i.e. time increment in nanoseconds, corresponding to one tick of this session when
    * in running mode.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param sessionDTSeconds the time increment in nanoseconds per running tick.
    * @see SessionMode
    */
   public void setSessionDTNanoseconds(long sessionDTNanoseconds)
   {
      if (this.sessionDTNanoseconds.get() == sessionDTNanoseconds)
         return;

      this.sessionDTNanoseconds.set(sessionDTNanoseconds);
      scheduleSessionTask(getActiveMode());
   }

   /**
    * Sets the initial size of this session's buffer.
    * <p>
    * Unlike {@link #submitBufferSizeRequest(Integer)} or
    * {@link #submitBufferSizeRequestAndWait(Integer)}, this method will change the buffer size only
    * the first time it is invoked. The subsequent calls will be ignored.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param bufferSize the initial size of the buffer. Default value
    *                   {@value #DEFAULT_INITIAL_BUFFER_SIZE}.
    * @return {@code true} if the request is going through, {@code false} if it is being ignored.
    */
   public boolean initializeBufferSize(int bufferSize)
   {
      if (hasBufferSizeBeenInitialized)
         return false;
      submitBufferSizeRequest(bufferSize);
      return true;
   }

   /**
    * Sets the initial record period in number of ticks for this session.
    * <p>
    * Unlike {@link #setBufferRecordTickPeriod(int)}, this method will change the property only the
    * first time it is invoked. The subsequent calls will be ignored.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param bufferRecordTickPeriod the period in number of ticks that data should be stored in the
    *                               buffer. Default value {@value #DEFAULT_BUFFER_RECORD_TICK_PERIOD}.
    * @return {@code true} if the request is going through, {@code false} if it is being ignored.
    */
   public boolean initializeBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      if (hasBufferRecordPeriodBeenInitialized)
         return false;
      setBufferRecordTickPeriod(bufferRecordTickPeriod);
      return true;
   }

   /**
    * Sets whether or not the {@link SessionMode#RUNNING} mode of this session should be capped to be
    * running no faster than real-time.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param runAtRealTimeRate {@code true} to cap the running mode at real-time rate, {@code false} to
    *                          let the running mode run as fast as possible. Default value
    *                          {@value #DEFAULT_RUN_AT_REALTIME_RATE}.
    */
   public void submitRunAtRealTimeRate(boolean runAtRealTimeRate)
   {
      if (this.runAtRealTimeRate.get() == runAtRealTimeRate)
         return;

      this.runAtRealTimeRate.set(runAtRealTimeRate);
      if (getActiveMode() == SessionMode.RUNNING)
         scheduleSessionTask(getActiveMode());
   }

   /**
    * Sets the speed at which the {@link SessionMode#PLAYBACK} should play back the buffered data.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * 
    * @param realTimeRate the real-time factor for playing back data in the buffer. Default value
    *                     {@value #DEFAULT_PLAYBACK_REALTIME_RATE}.
    */
   public void submitPlaybackRealTimeRate(double realTimeRate)
   {
      if (playbackRealTimeRate.get().doubleValue() == realTimeRate)
         return;

      playbackRealTimeRate.set(Double.valueOf(realTimeRate));
      if (getActiveMode() == SessionMode.PLAYBACK)
         scheduleSessionTask(getActiveMode());
   }

   /**
    * Sets the period, in number of ticks, at which data should be recorded into the buffer.
    * <p>
    * A larger value allows to store data over longer period of time.
    * </p>
    * 
    * @param bufferRecordTickPeriod the period in number of ticks that data should be stored in the
    *                               buffer. Default value {@value #DEFAULT_BUFFER_RECORD_TICK_PERIOD}.
    */
   public void setBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      hasBufferRecordPeriodBeenInitialized = true;
      if (bufferRecordTickPeriod == this.bufferRecordTickPeriod.get())
         return;
      this.bufferRecordTickPeriod.set(Math.max(1, bufferRecordTickPeriod));
   }

   /**
    * Sets the period used to control the rate at which the GUI will be refreshed.
    * <p>
    * This is particularly useful for instance when reading a log, increasing this period allows to
    * increase the speed at which we can read the log, also useful for the remote session to reduce the
    * computational induced by the GUI while receiving a high-bandwidth stream of data.
    * </p>
    * 
    * @param publishPeriod period in nanoseconds at which the buffer data is publish while in
    *                      {@link SessionMode#RUNNING} mode. Default value
    *                      {@value #DEFAULT_BUFFER_PUBLISH_PERIOD}.
    */
   public void setDesiredBufferPublishPeriod(long publishPeriod)
   {
      desiredBufferPublishPeriod.set(publishPeriod);
   }

   /**
    * Requests the buffer to be cropped.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise. This request will cancel any other requests submitted at the same time.
    * </p>
    * 
    * @param cropBufferRequest the request.
    * @see CropBufferRequest
    */
   public void submitCropBufferRequest(CropBufferRequest cropBufferRequest)
   {
      pendingCropBufferRequest.submit(cropBufferRequest);
   }

   /**
    * Requests to fill in the buffer entirely or partially with zeros or the value stored in each
    * {@link YoVariable}.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param fillBufferRequest the request.
    */
   public void submitFillBufferRequest(FillBufferRequest fillBufferRequest)
   {
      pendingFillBufferRequest.submit(fillBufferRequest);
   }

   /**
    * Requests to change the size the of the buffer.
    * <p>
    * This is typically used to increased the buffer size. To decrease the buffer size, it is
    * recommended to use a crop request that provides better control on the data being preserved.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferSizeRequest
    * @see #submitCropBufferRequest(CropBufferRequest)
    */
   public void submitBufferSizeRequest(Integer bufferSizeRequest)
   {
      pendingBufferSizeRequest.submit(bufferSizeRequest);
      hasBufferSizeBeenInitialized = true;
   }

   /**
    * Requests to move the current buffer index, i.e. reading/writing position.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferIndexRequest the current index to go to.
    */
   public void submitBufferIndexRequest(Integer bufferIndexRequest)
   {
      pendingBufferIndexRequest.submit(bufferIndexRequest);
   }

   /**
    * Requests to increment the current buffer index, i.e. reading/writing position, by a given step
    * size.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param incrementBufferIndexRequest the increment to apply to the current buffer index.
    */
   public void submitIncrementBufferIndexRequest(Integer incrementBufferIndexRequest)
   {
      pendingIncrementBufferIndexRequest.submit(incrementBufferIndexRequest);
   }

   /**
    * Requests to decrement the current buffer index, i.e. reading/writing position, by a given step
    * size.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param decrementBufferIndexRequest the decrement to apply to the current buffer index.
    */
   public void submitDecrementBufferIndexRequest(Integer decrementBufferIndexRequest)
   {
      pendingDecrementBufferIndexRequest.submit(decrementBufferIndexRequest);
   }

   /**
    * Requests to set the buffer in-point, i.e. the first index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferInPointIndexRequest the new index for the in-point.
    */
   public void submitBufferInPointIndexRequest(Integer bufferInPointIndexRequest)
   {
      pendingBufferInPointIndexRequest.submit(bufferInPointIndexRequest);
   }

   /**
    * Requests to set the buffer out-point, i.e. the last index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferOutPointIndexRequest the new index for the out-point.
    */
   public void submitBufferOutPointIndexRequest(Integer bufferOutPointIndexRequest)
   {
      pendingBufferOutPointIndexRequest.submit(bufferOutPointIndexRequest);
   }

   /**
    * Requests to export this session's data to file.
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param sessionDataExportRequest the request.
    * @see SessionDataExportRequest
    */
   public void submitSessionDataExportRequest(SessionDataExportRequest sessionDataExportRequest)
   {
      pendingDataExportRequest.submit(sessionDataExportRequest);
   }

   /**
    * Requests the buffer to be cropped.
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise. This request will cancel any other requests submitted at the same time.
    * </p>
    * 
    * @param cropBufferRequest the request.
    * @see CropBufferRequest
    */
   public void submitCropBufferRequestAndWait(CropBufferRequest cropBufferRequest)
   {
      if (hasSessionStarted())
      {
         pendingCropBufferRequest.submitAndWait(cropBufferRequest);
      }
      else
      {
         pendingCropBufferRequest.submit(cropBufferRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to fill in the buffer entirely or partially with zeros or the value stored in each
    * {@link YoVariable}.
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param fillBufferRequest the request.
    */
   public void submitFillBufferRequestAndWait(FillBufferRequest fillBufferRequest)
   {
      if (hasSessionStarted())
      {
         pendingFillBufferRequest.submitAndWait(fillBufferRequest);
      }
      else
      {
         pendingFillBufferRequest.submit(fillBufferRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to change the size the of the buffer.
    * <p>
    * This is typically used to increased the buffer size. To decrease the buffer size, it is
    * recommended to use a crop request that provides better control on the data being preserved.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferSizeRequest
    * @see #submitCropBufferRequest(CropBufferRequest)
    */
   public void submitBufferSizeRequestAndWait(Integer bufferSizeRequest)
   {
      hasBufferSizeBeenInitialized = true;
      if (hasSessionStarted())
      {
         pendingBufferSizeRequest.submitAndWait(bufferSizeRequest);
      }
      else
      {
         pendingBufferSizeRequest.submit(bufferSizeRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to move the current buffer index, i.e. reading/writing position.
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferIndexRequest the current index to go to.
    */
   public void submitBufferIndexRequestAndWait(Integer bufferIndexRequest)
   {
      if (hasSessionStarted())
      {
         pendingBufferIndexRequest.submitAndWait(bufferIndexRequest);
      }
      else
      {
         pendingBufferIndexRequest.submit(bufferIndexRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to increment the current buffer index, i.e. reading/writing position, by a given step
    * size.
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param incrementBufferIndexRequest the increment to apply to the current buffer index.
    */
   public void submitIncrementBufferIndexRequestAndWait(Integer incrementBufferIndexRequest)
   {
      if (hasSessionStarted())
      {
         pendingIncrementBufferIndexRequest.submitAndWait(incrementBufferIndexRequest);
      }
      else
      {
         pendingIncrementBufferIndexRequest.submit(incrementBufferIndexRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to decrement the current buffer index, i.e. reading/writing position, by a given step
    * size.
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param decrementBufferIndexRequest the decrement to apply to the current buffer index.
    */
   public void submitDecrementBufferIndexRequestAndWait(Integer decrementBufferIndexRequest)
   {
      if (hasSessionStarted())
      {
         pendingDecrementBufferIndexRequest.submitAndWait(decrementBufferIndexRequest);
      }
      else
      {
         pendingDecrementBufferIndexRequest.submit(decrementBufferIndexRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to set the buffer in-point, i.e. the first index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferInPointIndexRequest the new index for the in-point.
    */
   public void submitBufferInPointIndexRequestAndWait(Integer bufferInPointIndexRequest)
   {
      if (hasSessionStarted())
      {
         pendingBufferInPointIndexRequest.submitAndWait(bufferInPointIndexRequest);
      }
      else
      {
         pendingBufferInPointIndexRequest.submit(bufferInPointIndexRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to set the buffer out-point, i.e. the last index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param bufferOutPointIndexRequest the new index for the out-point.
    */
   public void submitBufferOutPointIndexRequestAndWait(Integer bufferOutPointIndexRequest)
   {
      if (hasSessionStarted())
      {
         pendingBufferOutPointIndexRequest.submitAndWait(bufferOutPointIndexRequest);
      }
      else
      {
         pendingBufferOutPointIndexRequest.submit(bufferOutPointIndexRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Requests to export this session's data to file.
    * <p>
    * This is a blocking operation and will return only when done. If the internal thread is not
    * running, this operation is performed immediately.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param sessionDataExportRequest the request.
    * @see SessionDataExportRequest
    */
   public void submitSessionDataExportRequestAndWait(SessionDataExportRequest sessionDataExportRequest)
   {
      if (hasSessionStarted())
      {
         pendingDataExportRequest.submitAndWait(sessionDataExportRequest);
      }
      else
      {
         pendingDataExportRequest.submit(sessionDataExportRequest);
         processBufferRequests(true);
      }
   }

   /**
    * Starts the internal thread of this session running the current session mode.
    * 
    * @return {@code true} if the thread has started, {@code false} if it could not be started, e.g. it
    *         was already started or the session was shutdown.
    */
   public boolean startSessionThread()
   {
      if (sessionThreadStarted)
      {
         LogTools.info("Session already started.");
         return false;
      }

      if (isSessionShutdown)
      {
         LogTools.error("Session has been shutdown.");
         return false;
      }

      LogTools.trace("Starting session's thread");
      sessionThreadStarted = true;
      scheduleSessionTask(getActiveMode());
      return true;
   }

   /**
    * Stops the internal thread of this session without notifying any of the listeners.
    * <p>
    * After calling this method, this session stops operating but it can resume from it by calling
    * {@link #startSessionThread()}.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done.
    * </p>
    * 
    * @return {@code true} if the thread has stopped, {@code false} if it could not be stopped, e.g. it
    *         was already stopped or the session was shutdown.
    */
   public boolean stopSessionThread()
   {
      if (!sessionThreadStarted)
         return false;
      if (isSessionShutdown)
         return false;

      if (activePeriodicTask != null)
      {
         activePeriodicTask.stopAndWait();
         activePeriodicTask = null;
      }

      sessionThreadStarted = false;

      return true;
   }

   /**
    * Shuts down this session permanently, it becomes unusable.
    * <p>
    * This method notifies the shutdown listeners and performs a memory cleanup.
    * </p>
    */
   public void shutdownSession()
   {
      if (isSessionShutdown)
         return;

      isSessionShutdown = true;

      shutdownListeners.forEach(Runnable::run);

      LogTools.info("Shutting down {}: {}", getClass().getSimpleName(), getSessionName());
      sessionThreadStarted = false;

      if (activePeriodicTask != null)
      {
         activePeriodicTask.stopAndWait();
         activePeriodicTask = null;
      }

      sessionTopicListenerManagers.forEach(SessionTopicListenerManager::detachFromMessager);
      sessionTopicListenerManagers.clear();
      sharedBuffer.dispose();
      rootRegistry.clear();

      executorService.shutdown();
      inertialFrame.removeListeners();
      inertialFrame.clearChildren();
   }

   private void scheduleSessionTask(SessionMode sessionMode)
   {
      scheduleSessionTask(sessionMode, null);
   }

   private void scheduleSessionTask(SessionMode newMode, SessionModeTransition transition)
   {
      SessionMode previousMode = activeMode.get();

      preSessionModeChangeListeners.forEach(listener -> listener.onChange(previousMode, newMode));

      if (!sessionThreadStarted)
      {
         // This is to allow running simulation without using the internal session thread.

         if (newMode != previousMode)
         {
            firstRunTick = true;
            firstPauseTick = true;
         }

         activeMode.set(newMode);
         sessionModeChangeListeners.forEach(listener -> listener.onChange(previousMode, newMode));
         reportActiveMode();
         return;
      }

      if (activePeriodicTask != null)
      {
         activePeriodicTask.stopAndWait();
         activePeriodicTask = null;
      }

      if (isSessionShutdown)
         return;

      if (newMode != previousMode)
      {
         firstRunTick = true;
         firstPauseTick = true;
      }

      runActualDT.reset();
      playbackActualDT.reset();
      pauseActualDT.reset();

      Runnable command;
      Runnable sessionModeTask = sessionModeToTaskMap.get(newMode);

      if (transition == null)
      {
         command = sessionModeTask;
      }
      else
      {
         Objects.requireNonNull(transition.getNextMode(), "nextMode argument is required when providing a terminalCondition");

         command = new Runnable()
         {
            private boolean terminated = false;

            @Override
            public void run()
            {
               sessionModeTask.run();

               if (terminated)
                  return;

               terminated = transition.isDone();

               if (terminated)
               {
                  executorService.execute(() ->
                  {
                     setSessionMode(transition.getNextMode());
                     transition.notifyTransitionComplete();
                  });
               }
            }
         };
      }

      if (isSessionShutdown)
         return;

      activePeriodicTask = new PeriodicTaskWrapper(command, computeThreadPeriod(newMode), TimeUnit.NANOSECONDS);
      activeMode.set(newMode);
      schedulingSessionMode(previousMode, newMode);
      executorService.execute(activePeriodicTask);
      sessionModeChangeListeners.forEach(listener -> listener.onChange(previousMode, newMode));
      reportActiveMode();
   }

   /**
    * Overrides this method to implement any specific transition action to be executed right before the
    * new mode starts running.
    * 
    * @param previousMode the previous mode. At this point, it is not longer running.
    * @param newMode      the new mode to run next, it is not yet running.
    */
   protected void schedulingSessionMode(SessionMode previousMode, SessionMode newMode)
   {
   }

   /**
    * Calculates the period in nanoseconds the periodic thread should be running at given the session
    * mode.
    */
   private long computeThreadPeriod(SessionMode mode)
   {
      switch (mode)
      {
         case RUNNING:
            return computeRunTaskPeriod();
         case PAUSE:
            return computePauseTaskPeriod();
         case PLAYBACK:
            return computePlaybackTaskPeriod();
         default:
            throw new UnsupportedOperationException("Unhandled session mode: " + mode);
      }
   }

   /**
    * Overrides the task to run for a given mode.
    * <p>
    * Here are the default tasks:
    * <ul>
    * <li>{@link SessionMode#RUNNING}: the task is {@link #runTick()},
    * <li>{@link SessionMode#PLAYBACK}: the task is {@link #playbackTick()},
    * <li>{@link SessionMode#PAUSE}: the task is {@link #pauseTick()}.
    * </ul>
    * </p>
    */
   protected void setSessionModeTask(SessionMode sessionMode, Runnable runnable)
   {
      sessionModeToTaskMap.put(sessionMode, runnable);
   }

   /** Reports the current {@link SessionProperties} to the listeners. */
   private void reportActiveMode()
   {
      for (Consumer<SessionProperties> listener : sessionPropertiesListeners)
         listener.accept(getSessionProperties());
   }

   /** Creates new {@link SessionProperties} with the current properties. */
   public SessionProperties getSessionProperties()
   {
      return new SessionProperties(activeMode.get(),
                                   runAtRealTimeRate.get(),
                                   playbackRealTimeRate.get().doubleValue(),
                                   sessionDTNanoseconds.get(),
                                   bufferRecordTickPeriod.get());
   }

   /**
    * Requests this session to be reinitialized as soon as possible.
    * <p>
    * This can be used to reinitialize the physics engine of a simulation for instance.
    * </p>
    * <p>
    * This is a non-blocking operation and schedules the change to be performed as soon as possible.
    * </p>
    */
   public void reinitializeSession()
   {
      sessionInitialized = false;
   }

   /**
    * Called when starting this session regardless of the initial mode.
    */
   protected void initializeSession()
   {
   }

   /**
    * Calculates the period in nanoseconds the periodic thread should be running at for
    * {@link SessionMode#RUNNING}.
    */
   protected long computeRunTaskPeriod()
   {
      return runAtRealTimeRate.get() ? sessionDTNanoseconds.get() : 1L;
   }

   /**
    * Stuff that needs to be done no matter the active session mode.
    */
   protected void doGeneric(SessionMode currentMode)
   {
      if (!sessionInitialized)
      {
         initializeSession();
         // When running simulation, the session starts in PAUSE, writing in the buffer allows to write the robot initial state.
         sharedBuffer.writeBuffer();
         sessionInitialized = true;
      }

      long currentTimestamp = System.nanoTime();

      if (currentTimestamp - lastSessionPropertiesPublishTimestamp > sessionPropertiesPublishPeriod)
      {
         lastSessionPropertiesPublishTimestamp = currentTimestamp;
         reportActiveMode();
      }
   }

   /**
    * Counter used to keep track of when the data should be written into the buffer according to the
    * user defined period {@link #bufferRecordTickPeriod}.
    */
   protected int nextRunBufferRecordTickCounter = 0;

   /**
    * Performs a single tick for the session mode {@link SessionMode#RUNNING}.
    * <p>
    * In order, this method calls:
    * <ol>
    * <li>{@link #doGeneric(SessionMode)}: generic set of actions regardless of the current mode,
    * <li>{@link #initializeRunTick()}: prepares the buffer doing actual computation,
    * <li>{@link #doSpecificRunTick()}: performs the actual computation of the run tick, for instance
    * runs a tick of the physics engine when working with a simulation session,
    * <li>{@link #finalizeRunTick(boolean)}: performs buffer operations to finalize the tick, such as
    * incrementing the current index in the buffer, and publishes data to listeners.
    * </ol>
    * </p>
    * <p>
    * If {@link #doSpecificRunTick()} results in an exception being thrown, the session mode will be
    * automatically changed to {@link SessionMode#PAUSE}.
    * </p>
    * 
    * @return {@code true} if the tick was run successfully, {@code false} if an exception was caught
    *         in {@link #doSpecificRunTick()}, e.g. the simulation failed or the controller crashed.
    */
   public boolean runTick()
   {
      runTimer.start();
      runActualDT.update();

      doGeneric(SessionMode.RUNNING);

      runInitializeTimer.start();
      initializeRunTick();
      runInitializeTimer.stop();

      boolean caughtException;
      try
      {
         runSpecificTimer.start();
         time.set(doSpecificRunTick());
         runSpecificTimer.stop();
         caughtException = false;
      }
      catch (Throwable e)
      {
         e.printStackTrace();
         runThrowableListeners.forEach(listener -> listener.accept(e));
         caughtException = true;
      }

      jvmStatisticsGenerator.update();

      runFinalizeTimer.start();
      finalizeRunTick(false);
      runFinalizeTimer.stop();

      runTimer.stop();

      if (caughtException)
         setSessionMode(SessionMode.PAUSE);

      return !caughtException;
   }

   /**
    * Prepares the buffer before performing actual computation for the session mode
    * {@link SessionMode#RUNNING}.
    * <p>
    * It is at this time that we are processing the changes requested by the user through
    * {@link LinkedYoVariable}s or via the {@link Messager}. Some operations, such as cropping the
    * buffer, are not allowed in this mode and will be ignored.
    * </p>
    */
   protected void initializeRunTick()
   {
      if (firstRunTick)
      {
         sharedBuffer.incrementBufferIndex(true);
         sharedBuffer.processLinkedPushRequests(false);
         nextRunBufferRecordTickCounter = 0;
         firstRunTick = false;
      }
      else if (nextRunBufferRecordTickCounter <= 0)
      {
         sharedBuffer.incrementBufferIndex(true);
         sharedBuffer.processLinkedPushRequests(false);
      }
   }

   /**
    * Performs action specific to the implementation of this session when running
    * {@link SessionMode#RUNNING}.
    *
    * @return the current time in seconds.
    */
   protected abstract double doSpecificRunTick();

   /**
    * Performs buffer operations to finalize a run tick, such as incrementing the current index and
    * writing the {@link YoVariable} values into the buffer, and publishes data to the listeners.
    * <p>
    * The data contained in the {@code YoVariable}s is written in the buffer only every
    * {@link #bufferRecordTickPeriod} ticks, unless the given flag {@code forceWriteBuffer} is set to
    * {@code true}.
    * </p>
    * 
    * @param forceWriteBuffer when {@code true} the buffer will be updated regardless of the
    *                         {@code nextRunBufferRecordTickCounter} value.
    */
   protected void finalizeRunTick(boolean forceWriteBuffer)
   {
      boolean writeBuffer = nextRunBufferRecordTickCounter <= 0;

      if (!writeBuffer && forceWriteBuffer)
      {
         sharedBuffer.incrementBufferIndex(true);
         writeBuffer = true;
      }

      if (writeBuffer)
      {
         sharedBuffer.writeBuffer();

         long currentTimestamp = System.nanoTime();

         if (currentTimestamp - lastPublishedBufferTimestamp > desiredBufferPublishPeriod.get())
         {
            sharedBuffer.prepareLinkedBuffersForPull();
            lastPublishedBufferTimestamp = currentTimestamp;
         }

         processBufferRequests(false);
         publishBufferProperties(sharedBuffer.getProperties());

         nextRunBufferRecordTickCounter = Math.max(1, bufferRecordTickPeriod.get());
      }

      nextRunBufferRecordTickCounter--;
   }

   /**
    * Indicates whether the last call of {@link #runTick()} has resulted in writing the
    * {@code YoVariable}s into the buffer.
    * 
    * @return {@code true} if the buffer was updated, {@code false} otherwise.
    */
   public boolean hasWrittenBufferInLastRunTick()
   {
      return nextRunBufferRecordTickCounter == Math.max(1, bufferRecordTickPeriod.get()) - 1;
   }

   /**
    * Calculates the period in nanoseconds the periodic thread should be running at for
    * {@link SessionMode#PLAYBACK}.
    */
   protected long computePlaybackTaskPeriod()
   {
      long timeIncrement = sessionDTNanoseconds.get() * bufferRecordTickPeriod.get();

      if (playbackRealTimeRate.get().doubleValue() <= 0.5)
      {
         stepSizePerPlaybackTick = 1;
         return (long) (timeIncrement / playbackRealTimeRate.get().doubleValue());
      }
      else
      {
         stepSizePerPlaybackTick = 2 * Math.max(1, (int) Math.floor(playbackRealTimeRate.get().doubleValue()));
         return (long) (timeIncrement * stepSizePerPlaybackTick / playbackRealTimeRate.get().doubleValue());
      }
   }

   /**
    * Performs a single tick for the session mode {@link SessionMode#PLAYBACK}.
    * <p>
    * In order, this method calls:
    * <ol>
    * <li>{@link #doGeneric(SessionMode)}: generic set of actions regardless of the current mode,
    * <li>{@link #initializePlaybackTick()}: prepares the buffer and reads the buffer at the current
    * index to update the {@link YoVariable} values,
    * <li>{@link #doSpecificPlaybackTick()}: performs the actual computation of the playback tick,
    * typically nothing happens here as we're only playing through the buffered data,
    * <li>{@link #finalizePlaybackTick()}: performs buffer operations to finalize the tick and
    * publishes data to listeners.
    * </ol>
    * </p>
    * <p>
    * If {@link #doSpecificPlaybackTick()} results in an exception being thrown, the session mode will
    * be automatically changed to {@link SessionMode#PAUSE}.
    * </p>
    */
   public void playbackTick()
   {
      playbackTimer.start();
      playbackActualDT.update();

      doGeneric(SessionMode.PLAYBACK);

      initializePlaybackTick();

      boolean caughtException;
      try
      {
         doSpecificPlaybackTick();
         caughtException = false;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         playbackThrowableListeners.forEach(listener -> listener.accept(e));
         caughtException = true;
      }

      finalizePlaybackTick();

      playbackTimer.stop();

      if (caughtException)
         setSessionMode(SessionMode.PAUSE);
   }

   /**
    * Ignores all {@link YoVariable} value changes submitted by the user via {@link LinkedYoVariable}s
    * and reads the buffer data to update the {@link YoVariable} values.
    */
   protected void initializePlaybackTick()
   {
      sharedBuffer.flushLinkedPushRequests();
      sharedBuffer.readBuffer();
   }

   /**
    * Override to performs specific operations during a playback tick. Typically nothing happens here
    * as we are only reading through the buffer data.
    */
   protected void doSpecificPlaybackTick()
   {

   }

   /**
    * Performs buffer operation to finalize a playback tick, such as incrementing the current index and
    * publishes data to the listeners.
    */
   protected void finalizePlaybackTick()
   {
      long currentTimestamp = System.nanoTime();

      if (currentTimestamp - lastPublishedBufferTimestamp > desiredBufferPublishPeriod.get())
      {
         sharedBuffer.prepareLinkedBuffersForPull();
         lastPublishedBufferTimestamp = currentTimestamp;
      }

      sharedBuffer.incrementBufferIndex(false, stepSizePerPlaybackTick);
      processBufferRequests(false);
      publishBufferProperties(sharedBuffer.getProperties());
   }

   /**
    * Calculates the period in nanoseconds the periodic thread should be running at for
    * {@link SessionMode#PAUSE}.
    */
   protected long computePauseTaskPeriod()
   {
      return Conversions.secondsToNanoseconds(0.01);
   }

   /**
    * Performs a single tick for the session mode {@link SessionMode#PAUSE}.
    * <p>
    * In order, this method calls:
    * <ol>
    * <li>{@link #doGeneric(SessionMode)}: generic set of actions regardless of the current mode,
    * <li>{@link #initializePauseTick()}: initialize the buffer, mostly about fetching the changes
    * requested by the user,
    * <li>{@link #doSpecificPauseTick()}: this should pretty much always do nothing,
    * <li>{@link #finalizePauseTick()}: buffer operations to finalize the tick and publishes data to
    * the listeners.
    * </ol>
    * </p>
    */
   public void pauseTick()
   {
      pauseTimer.start();
      pauseActualDT.update();

      doGeneric(SessionMode.PAUSE);

      boolean shouldReadBuffer = initializePauseTick();
      shouldReadBuffer |= doSpecificPauseTick();
      finalizePauseTick(shouldReadBuffer);

      pauseTimer.stop();
   }

   /**
    * Prepares the buffer for a pause tick, this is mainly about fetching the user requested changes
    * onto the buffer.
    * 
    * @return whether the buffer has been modified and should be read at the end of the pause tick.
    */
   protected boolean initializePauseTick()
   {
      boolean shouldReadBuffer = firstPauseTick;
      firstPauseTick = false;
      shouldReadBuffer |= sharedBuffer.processLinkedPushRequests(true);
      shouldReadBuffer |= processBufferRequests(true);
      if (!shouldReadBuffer)
         shouldReadBuffer = sharedBuffer.hasRequestPending();

      return shouldReadBuffer;
   }

   /**
    * Should pretty much always do nothing.
    * <p>
    * But if you really want to, you can override this method to perform custom operations during a
    * pause tick.
    * </p>
    * 
    * @return whether the buffer has been modified and should be read at the end of the pause tick.
    */
   protected boolean doSpecificPauseTick()
   {
      return false;
   }

   /**
    * Performs buffer operations to finalize a pause tick and publishes data to publisher.
    * 
    * @param shouldReadBuffer whether to read the buffer at the current index and update the
    *                         {@link YoVariable} values.
    */
   protected void finalizePauseTick(boolean shouldReadBuffer)
   {
      if (shouldReadBuffer)
         sharedBuffer.readBuffer();

      long currentTimestamp = System.nanoTime();

      if (currentTimestamp - lastPublishedBufferTimestamp > desiredBufferPublishPeriod.get())
      {
         sharedBuffer.prepareLinkedBuffersForPull();
         lastPublishedBufferTimestamp = currentTimestamp;
      }

      publishBufferProperties(sharedBuffer.getProperties());
   }

   /**
    * Submits the given buffer properties to all the listeners.
    * 
    * @param bufferProperties the properties to be submitted to the listeners.
    */
   protected void publishBufferProperties(YoBufferPropertiesReadOnly bufferProperties)
   {
      for (Consumer<YoBufferPropertiesReadOnly> listener : currentBufferPropertiesListeners)
      {
         listener.accept(bufferProperties.copy());
      }
   }

   /**
    * Handles user requests for modifying the buffer.
    * <p>
    * Operations handled here are: changing indices (current, in-point, out-point), resizing, cropping,
    * filling.
    * </p>
    * 
    * @param bufferChangesPermitted indicates whether all operations onto the buffer are permitted
    *                               ({@code true)} or only a minimal subset is permitted
    *                               ({@code false}). When in {@link SessionMode#RUNNING} or
    *                               {@link SessionMode#PLAYBACK}, changing the indices is not allowed
    *                               for instance.
    * @return whether the buffer has been modified.
    */
   protected boolean processBufferRequests(boolean bufferChangesPermitted)
   {
      boolean hasBufferBeenUpdated = false;

      CropBufferRequest newCropRequest = pendingCropBufferRequest.poll();

      if (bufferChangesPermitted)
      {
         if (newCropRequest != null)
         {
            sharedBuffer.cropBuffer(newCropRequest);
            hasBufferBeenUpdated = true;
            bufferChangesPermitted = false;
         }
      }

      Integer newIndex = pendingBufferIndexRequest.poll();
      Integer newInPoint = pendingBufferInPointIndexRequest.poll();
      Integer newOutPoint = pendingBufferOutPointIndexRequest.poll();
      Integer incStepSize = pendingIncrementBufferIndexRequest.poll();
      Integer decStepSize = pendingDecrementBufferIndexRequest.poll();
      Integer newSize = pendingBufferSizeRequest.poll();
      FillBufferRequest fillBufferRequest = pendingFillBufferRequest.poll();
      SessionDataExportRequest dataExportRequest = pendingDataExportRequest.poll();

      if (bufferChangesPermitted)
      {
         if (newIndex != null)
            hasBufferBeenUpdated |= sharedBuffer.setCurrentIndex(newIndex.intValue());

         if (newInPoint != null)
            hasBufferBeenUpdated |= sharedBuffer.setInPoint(newInPoint.intValue());

         if (newOutPoint != null)
            hasBufferBeenUpdated |= sharedBuffer.setOutPoint(newOutPoint.intValue());

         if (incStepSize != null)
         {
            sharedBuffer.incrementBufferIndex(false, incStepSize.intValue());
            hasBufferBeenUpdated = true;
         }

         if (decStepSize != null)
         {
            sharedBuffer.decrementBufferIndex(decStepSize.intValue());
            hasBufferBeenUpdated = true;
         }

         if (fillBufferRequest != null)
         {
            sharedBuffer.fillBuffer(fillBufferRequest);
            hasBufferBeenUpdated = true;
         }

         if (dataExportRequest != null)
         {
            try
            {
               SessionIOTools.exportSessionData(this, dataExportRequest);
            }
            catch (JAXBException | IOException | URISyntaxException e)
            {
               e.printStackTrace();
            }
         }
      }

      if (newSize != null)
         hasBufferBeenUpdated |= sharedBuffer.resizeBuffer(newSize.intValue());

      return hasBufferBeenUpdated;
   }

   /**
    * Gets the variable holding the current time (in seconds) in this session.
    * 
    * @return the current time (in seconds) variable.
    */
   public YoDouble getTime()
   {
      return time;
   }

   /**
    * Gets this session's root registry.
    * 
    * @return the root registry.
    */
   public YoRegistry getRootRegistry()
   {
      return rootRegistry;
   }

   /**
    * Returns whether this session's thread has been started.
    * 
    * @return {@code true} if the session thread has been started.
    */
   public boolean hasSessionStarted()
   {
      return sessionThreadStarted;
   }

   /**
    * Returns whether this session has been shutdown and is thus unusable.
    * 
    * @return {@code true} if this session has been shutdown.
    */
   public boolean isSessionShutdown()
   {
      return isSessionShutdown;
   }

   /**
    * Gets the current mode of this session.
    * 
    * @return the active mode.
    */
   public SessionMode getActiveMode()
   {
      return activeMode.get();
   }

   /**
    * Whether the {@link SessionMode#RUNNING} mode is capped to run no faster that real-time.
    * 
    * @return {@code true} if the running mode is capped to run no faster than real-time.
    */
   public boolean getRunAtRealTimeRate()
   {
      return runAtRealTimeRate.get();
   }

   /**
    * The speed at which the {@link SessionMode#PLAYBACK} should play back the buffered data.
    * 
    * @return real-time factor used for the playback.
    */
   public double getPlaybackRealTimeRate()
   {
      return playbackRealTimeRate.get().doubleValue();
   }

   /**
    * The number of times {@link #runTick()} should be called before saving the {@link YoVariable} data
    * into the buffer.
    * 
    * @return the period, in number of run ticks, at which the {@link YoVariable}s are saved into the
    *         buffer.
    */
   public int getBufferRecordTickPeriod()
   {
      return bufferRecordTickPeriod.get();
   }

   /**
    * The period in seconds at which data from the {@link YoVariable}s is written into the buffer.
    * 
    * @return the period, in seconds, at which the {@link YoVariable}s are saved into the buffer.
    */
   public double getBufferRecordTimePeriod()
   {
      return getBufferRecordTickPeriod() * getSessionDTSeconds();
   }

   /**
    * The time increment in seconds corresponding to the execution of one run tick.
    * 
    * @return the time increment in seconds per running tick.
    */
   public double getSessionDTSeconds()
   {
      return sessionDTNanoseconds.get() * 1.0e-9;
   }

   /**
    * The time increment in nanoseconds corresponding to the execution of one run tick.
    * 
    * @return the time increment in nanoseconds per running tick.
    */
   public long getSessionDTNanoseconds()
   {
      return sessionDTNanoseconds.get();
   }

   /**
    * The period at which the buffer is published to the listeners. The is mainly to control the GUI
    * refresh rate.
    * 
    * @return the buffer publish period in nanoseconds.
    */
   public long getDesiredBufferPublishPeriod()
   {
      return desiredBufferPublishPeriod.get();
   }

   /**
    * Gets the session name.
    * 
    * @return the session name.
    */
   public abstract String getSessionName();

   /**
    * The inertial frame. It is expected to be a root frame that is different to
    * {@link ReferenceFrame#getWorldFrame()}.
    * 
    * @return this session inertial frame.
    */
   public final ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   /**
    * Gets the list of all the robot definitions this session is handling. The number of robot
    * definitions is expected to be the same as the number of robots.
    * <p>
    * This list is notably used by the GUI to visualize the robots.
    * </p>
    * 
    * @return the robot definition list.
    */
   public abstract List<RobotDefinition> getRobotDefinitions();

   /**
    * Gets the list of the terrain definitions used to represent the static environment used during
    * this session.
    * <p>
    * This list is notably used by the GUI to visualize the environment.
    * </p>
    * 
    * @return the terrain object definition list.
    */
   public abstract List<TerrainObjectDefinition> getTerrainObjectDefinitions();

   /**
    * Gets the list of yoGraphic to be visualized with this session.
    * <p>
    * This list is notably used by the GUI to instantiate yoGraphics.
    * </p>
    * 
    * @return the yoGraphic definition list.
    */
   public List<YoGraphicDefinition> getYoGraphicDefinitions()
   {
      return Collections.emptyList();
   }

   /*
    * FIXME This implementation doesn't look right. This is a workaround for the fact that Robot
    * doesn't live in this project. It seems that Robot, LogSession, RemoteSession,
    * SimulationDataSession, and VisualizationSession should live in the session project.
    */
   /**
    * Override me to allow exporting robot states.
    * 
    * @param initialState when {@code true}, the state of the robot as of before performing the
    *                     run-tick operations.
    */
   public List<RobotStateDefinition> getCurrentRobotStateDefinitions(boolean initialState)
   {
      return null;
   }

   /**
    * Gets the instance of this session's buffer.
    * <p>
    * It is not recommended to access and operate directly on the buffer, prefer using
    * {@link #getLinkedYoVariableFactory()}.
    * </p>
    * 
    * @return the internal buffer.
    */
   public YoSharedBuffer getBuffer()
   {
      return sharedBuffer;
   }

   /**
    * Gets the current buffer properties.
    * 
    * @return the read-only properties of the buffer.
    */
   public YoBufferPropertiesReadOnly getBufferProperties()
   {
      return sharedBuffer.getProperties();
   }

   /**
    * Gets the factory for creating {@link LinkedYoVariable}s that can be used to operate safely with
    * this session buffer.
    * 
    * @return the linked yoVariable factory.
    */
   public LinkedYoVariableFactory getLinkedYoVariableFactory()
   {
      return sharedBuffer;
   }

   /**
    * Convenience class used to hook up a session with a {@link Messager}.
    * <p>
    * For internal use only.
    * </p>
    */
   private class SessionTopicListenerManager
   {
      private final Messager messager;

      private final TopicListener<CropBufferRequest> cropRequestListener = Session.this::submitCropBufferRequest;
      private final TopicListener<FillBufferRequest> fillRequestListener = Session.this::submitFillBufferRequest;
      private final TopicListener<Integer> currentIndexListener = Session.this::submitBufferIndexRequest;
      private final TopicListener<Integer> inPointIndexListener = Session.this::submitBufferInPointIndexRequest;
      private final TopicListener<Integer> outPointIndexListener = Session.this::submitBufferOutPointIndexRequest;
      private final TopicListener<Integer> incrementCurrentIndexListener = Session.this::submitIncrementBufferIndexRequest;
      private final TopicListener<Integer> decrementCurrentIndexListener = Session.this::submitDecrementBufferIndexRequest;
      private final TopicListener<Integer> currentBufferSizeListener = Session.this::submitBufferSizeRequest;
      private final TopicListener<Integer> initializeBufferSizeListener = Session.this::initializeBufferSize;

      // TODO Look into removing the SessionState enum, seems unnecessary
      private final TopicListener<SessionState> sessionCurrentStateListener = state ->
      {
         if (state == SessionState.ACTIVE)
            startSessionThread();
         else if (state == SessionState.INACTIVE)
            shutdownSession();
      };
      private final TopicListener<SessionMode> sessionCurrentModeListener = Session.this::setSessionMode;
      private final TopicListener<Long> sessionDTNanosecondsListener = Session.this::setSessionDTNanoseconds;
      private final TopicListener<Boolean> runAtRealTimeRateListener = Session.this::submitRunAtRealTimeRate;
      private final TopicListener<Double> playbackRealTimeRateListener = Session.this::submitPlaybackRealTimeRate;
      private final TopicListener<Integer> bufferRecordTickPeriodListener = Session.this::setBufferRecordTickPeriod;
      private final TopicListener<Integer> initializeBufferRecordTickPeriodListener = Session.this::initializeBufferRecordTickPeriod;
      private final TopicListener<SessionDataExportRequest> sessionDataExportRequestListener = Session.this::submitSessionDataExportRequest;

      private final Consumer<YoBufferPropertiesReadOnly> bufferPropertiesListener = createBufferPropertiesListener();
      private final Consumer<SessionProperties> sessionPropertiesListener = createSessionPropertiesListener();

      private SessionTopicListenerManager(Messager messager)
      {
         this.messager = messager;

         addCurrentBufferPropertiesListener(bufferPropertiesListener);

         messager.registerTopicListener(YoSharedBufferMessagerAPI.CropRequest, cropRequestListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.FillRequest, fillRequestListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.CurrentIndexRequest, currentIndexListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.InPointIndexRequest, inPointIndexListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.OutPointIndexRequest, outPointIndexListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.IncrementCurrentIndexRequest, incrementCurrentIndexListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.DecrementCurrentIndexRequest, decrementCurrentIndexListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.CurrentBufferSizeRequest, currentBufferSizeListener);
         messager.registerTopicListener(YoSharedBufferMessagerAPI.InitializeBufferSize, initializeBufferSizeListener);

         addSessionPropertiesListener(sessionPropertiesListener);

         messager.registerTopicListener(SessionMessagerAPI.SessionCurrentState, sessionCurrentStateListener);
         messager.registerTopicListener(SessionMessagerAPI.SessionCurrentMode, sessionCurrentModeListener);
         messager.registerTopicListener(SessionMessagerAPI.SessionDTNanoseconds, sessionDTNanosecondsListener);
         messager.registerTopicListener(SessionMessagerAPI.RunAtRealTimeRate, runAtRealTimeRateListener);
         messager.registerTopicListener(SessionMessagerAPI.PlaybackRealTimeRate, playbackRealTimeRateListener);
         messager.registerTopicListener(SessionMessagerAPI.BufferRecordTickPeriod, bufferRecordTickPeriodListener);
         messager.registerTopicListener(SessionMessagerAPI.InitializeBufferRecordTickPeriod, initializeBufferRecordTickPeriodListener);
         messager.registerTopicListener(SessionMessagerAPI.SessionDataExportRequest, sessionDataExportRequestListener);
      }

      private void detachFromMessager()
      {
         if (messager == null)
            return;

         messager.removeTopicListener(YoSharedBufferMessagerAPI.CropRequest, cropRequestListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.FillRequest, fillRequestListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.CurrentIndexRequest, currentIndexListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.InPointIndexRequest, inPointIndexListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.OutPointIndexRequest, outPointIndexListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.IncrementCurrentIndexRequest, incrementCurrentIndexListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.DecrementCurrentIndexRequest, decrementCurrentIndexListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.CurrentBufferSizeRequest, currentBufferSizeListener);
         messager.removeTopicListener(YoSharedBufferMessagerAPI.InitializeBufferSize, initializeBufferSizeListener);

         messager.removeTopicListener(SessionMessagerAPI.SessionCurrentState, sessionCurrentStateListener);
         messager.removeTopicListener(SessionMessagerAPI.SessionCurrentMode, sessionCurrentModeListener);
         messager.removeTopicListener(SessionMessagerAPI.SessionDTNanoseconds, sessionDTNanosecondsListener);
         messager.removeTopicListener(SessionMessagerAPI.RunAtRealTimeRate, runAtRealTimeRateListener);
         messager.removeTopicListener(SessionMessagerAPI.PlaybackRealTimeRate, playbackRealTimeRateListener);
         messager.removeTopicListener(SessionMessagerAPI.BufferRecordTickPeriod, bufferRecordTickPeriodListener);
         messager.removeTopicListener(SessionMessagerAPI.InitializeBufferRecordTickPeriod, initializeBufferRecordTickPeriodListener);
         messager.removeTopicListener(SessionMessagerAPI.SessionDataExportRequest, sessionDataExportRequestListener);
      }

      private Consumer<YoBufferPropertiesReadOnly> createBufferPropertiesListener()
      {
         return bufferProperties ->
         {
            if (!messager.isMessagerOpen())
               return;

            messager.submitMessage(YoSharedBufferMessagerAPI.CurrentBufferProperties, bufferProperties);
         };
      }

      private Consumer<SessionProperties> createSessionPropertiesListener()
      {
         return sessionProperties ->
         {
            if (!messager.isMessagerOpen())
               return;

            messager.submitMessage(SessionMessagerAPI.SessionCurrentMode, sessionProperties.getActiveMode());
            messager.submitMessage(SessionMessagerAPI.SessionDTNanoseconds, sessionProperties.getSessionDTNanoseconds());
            messager.submitMessage(SessionMessagerAPI.PlaybackRealTimeRate, sessionProperties.getPlaybackRealTimeRate());
            messager.submitMessage(SessionMessagerAPI.RunAtRealTimeRate, sessionProperties.isRunAtRealTimeRate());
            messager.submitMessage(SessionMessagerAPI.BufferRecordTickPeriod, sessionProperties.getBufferRecordTickPeriod());
         };
      }
   }

   /**
    * Interface to to implement a conditional based transition from one session mode to the next.
    * <p>
    * For instance, this transition be used to schedule a simulation of a fixed amount of time at the
    * end of which the pause mode should be entered.
    * </p>
    */
   public interface SessionModeTransition
   {
      /**
       * The next mode to switch to once {@link #isDone()} returns {@code true}.
       * 
       * @return the mode to transition to once the current mode is done.
       */
      SessionMode getNextMode();

      /**
       * Tests whether the current mode is done.
       * 
       * @return {@code true} if the current mode is done and should be terminated.
       */
      boolean isDone();

      /**
       * Overrides this method to be notified once the transition has been performed, i.e. the current
       * mode is done and terminated and the transition to the next mode has been scheduled.
       */
      default void notifyTransitionComplete()
      {
      }

      /**
       * Factory for a creating a condition based transition.
       * 
       * @param doneCondition the condition used to determine when the current mode is done and should be
       *                      terminated.
       * @param nextMode      the mode to switch to once the current mode is done.
       * @return the transition that can be used with
       *         {@link Session#setSessionMode(SessionMode, SessionModeTransition)}.
       */
      static SessionModeTransition newTransition(BooleanSupplier doneCondition, SessionMode nextMode)
      {
         return new SessionModeTransition()
         {
            @Override
            public boolean isDone()
            {
               return doneCondition.getAsBoolean();
            }

            @Override
            public SessionMode getNextMode()
            {
               return nextMode;
            }
         };
      }
   }

   /**
    * Interface for implementing a listener that is to be notified when a session changes modes.
    *
    * @see Session#addSessionModeChangeListener(SessionModeChangeListener)
    */
   public interface SessionModeChangeListener
   {
      /**
       * Notification of a mode change.
       * <p>
       * This method is called right after the new mode has been scheduled with the executor.
       * </p>
       * 
       * @param previousMode the mode that was running previously. It has been stopped at this time.
       * @param newMode      the mode that is about to run. It has been schedule with the executor and may
       *                     have started to run already.
       */
      void onChange(SessionMode previousMode, SessionMode newMode);
   }

   /**
    * Implementation of {@code Runnable} that runs a task at a periodic rate.
    * <p>
    * This implementation provides an API that allows to synchronize with the internal task state.
    * </p>
    */
   public static class PeriodicTaskWrapper implements Runnable
   {
      private static final int ONE_MILLION = 1000000;

      /** The task to run periodically. */
      private final Runnable task;
      /** The period at which the task should be run. */
      private final long periodInNanos;
      /** Whether the task is being run or not. */
      private final AtomicBoolean running = new AtomicBoolean(true);
      /** Indicates whether the task is done running or not. */
      private final AtomicBoolean isDone = new AtomicBoolean(false);
      private final CountDownLatch startedLatch = new CountDownLatch(1);
      private final CountDownLatch doneLatch = new CountDownLatch(1);

      private Thread owner;

      public PeriodicTaskWrapper(Runnable task, long period, TimeUnit timeUnit)
      {
         this.task = task;
         periodInNanos = timeUnit.toNanos(period);
      }

      public void stop()
      {
         running.set(false);
      }

      public boolean isDone()
      {
         return isDone.get();
      }

      public void stopAndWait()
      {
         stop();

         if (Thread.currentThread() == owner)
            return;

         try
         {
            doneLatch.await();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }

      public void waitUntilFirstTickDone()
      {
         if (Thread.currentThread() == owner)
            return;

         try
         {
            startedLatch.await();
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }

      @Override
      public void run()
      {
         if (owner == null)
            owner = Thread.currentThread();

         try
         {
            while (running.get())
            {
               long timeElapsed = System.nanoTime();
               task.run();
               timeElapsed = System.nanoTime() - timeElapsed;

               startedLatch.countDown();

               if (timeElapsed < periodInNanos)
               {
                  long nanos = periodInNanos - timeElapsed;
                  long millis = nanos / ONE_MILLION;
                  nanos -= millis * ONE_MILLION;

                  try
                  {
                     Thread.sleep(millis, (int) nanos);
                  }
                  catch (InterruptedException e)
                  {
                     e.printStackTrace();
                     running.set(false);
                  }
               }
            }
         }
         catch (Throwable e)
         {
            e.printStackTrace();
         }

         isDone.set(true);
         doneLatch.countDown();
      }
   }

   /**
    * This class is designed to be used for the field of a {@code Session} that can be modified through
    * the class API. This class offers to either submit a request to be handled asynchronously or
    * synchronously by blocking the calling thread until the request has been processed.
    */
   protected class SessionUserField<T>
   {
      private final AtomicReference<T> nonBlockingRequest;
      private final ConcurrentLinkedQueue<BlockingRequest> blockingRequests = new ConcurrentLinkedQueue<>();
      private T currentValue;

      public SessionUserField()
      {
         this(null);
      }

      public SessionUserField(T initialValue)
      {
         currentValue = initialValue;
         nonBlockingRequest = new AtomicReference<>(initialValue);
      }

      public T getCurrentValue()
      {
         return currentValue;
      }

      public void submit(T newValue)
      {
         nonBlockingRequest.set(newValue);
      }

      public void submitAndWait(T newValue)
      {
         BlockingRequest blockingRequest = new BlockingRequest(newValue);
         blockingRequests.add(blockingRequest);
         try
         {
            blockingRequest.latch.await();
         }
         catch (InterruptedException e)
         {
         }
      }

      public T poll()
      {
         if (blockingRequests.isEmpty())
         {
            return nonBlockingRequest.getAndSet(null);
         }
         else
         {
            return blockingRequests.poll().process();
         }
      }

      private class BlockingRequest
      {
         private final CountDownLatch latch = new CountDownLatch(1);
         private final T requestedValue;

         public BlockingRequest(T requestedValue)
         {
            this.requestedValue = requestedValue;
         }

         public T process()
         {
            try
            {
               return requestedValue;
            }
            finally
            {
               latch.countDown();
            }
         }
      }
   }
}
