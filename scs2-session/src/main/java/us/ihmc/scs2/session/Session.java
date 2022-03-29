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
   private static final int DEFAULT_INITIAL_BUFFER_SIZE = 8192;
   private static final int DEFAULT_BUFFER_RECORD_TICK_PERIOD = 1;
   private static final boolean DEFAULT_RUN_AT_REALTIME_RATE = false;
   private static final double DEFAULT_PLAYBACK_REALTIME_RATE = 2.0;
   private static final long DEFAULT_BUFFER_PUBLISH_PERIOD = -1L;

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
   protected final YoDouble time = new YoDouble("time", rootRegistry);
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
    * Map from one session tick to the time increment in the data.
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
   private final List<SessionModeChangeListener> sessionModeChangeListeners = new ArrayList<>();
   private final List<Consumer<SessionState>> sessionStateChangedListeners = new ArrayList<>();
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
   private boolean sessionStarted = false;
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
      this(SessionMode.PAUSE);
   }

   public Session(SessionMode initialMode)
   {
      this(ReferenceFrameTools.constructARootFrame("worldFrame"), initialMode);
   }

   public Session(ReferenceFrame inertialFrame)
   {
      this(inertialFrame, SessionMode.PAUSE);
   }

   public Session(ReferenceFrame inertialFrame, SessionMode initialMode)
   {
      this.inertialFrame = inertialFrame;

      activeMode.set(initialMode);
      rootRegistry.addChild(sessionRegistry);
      sessionRegistry.addChild(runRegistry);
      sessionRegistry.addChild(playbackRegistry);
      sessionRegistry.addChild(pauseRegistry);

      sessionModeToTaskMap.put(SessionMode.RUNNING, this::runTick);
      sessionModeToTaskMap.put(SessionMode.PLAYBACK, this::playbackTick);
      sessionModeToTaskMap.put(SessionMode.PAUSE, this::pauseTick);
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

   protected void setSessionMode(SessionMode sessionMode, SessionModeTransition transition)
   {
      SessionMode currentMode = activeMode.get();

      if (sessionMode != currentMode)
         scheduleSessionTask(sessionMode, transition);
   }

   /**
    * Adds a listener to be notified whenever this session changes mode.
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
    * Adds a listener to be notified whenever this session changes state, i.e. active or inactive for
    * when the session gets shutdown.
    * 
    * @param listener the listener to add.
    * @see SessionState
    */
   public void addSessionStateChangedListener(Consumer<SessionState> listener)
   {
      sessionStateChangedListeners.add(listener);
   }

   /**
    * Removes a listener previously registered to this session.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed, {@code false} if it could not be
    *         found.
    */
   public boolean removeSessionStateChangedListener(Consumer<SessionState> listener)
   {
      return sessionStateChangedListeners.remove(listener);
   }

   /**
    * Adds a listener to be notified whenever this session is about to shutdown. This listener will be
    * called before the session becomes inactive.
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
    * Unlike {@link #submitBufferRecordTickPeriod(int)}, this method will change the property only the
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
      submitBufferRecordTickPeriod(bufferRecordTickPeriod);
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
      scheduleSessionTask(getActiveMode());
   }

   /**
    * Sets the period, in number of ticks, at which data should be recorded into the buffer.
    * <p>
    * A larger value allows to store data over longer period of time.
    * </p>
    * <p>
    * This is a non-blocking operation and the change is applied immediately.
    * </p>
    * 
    * @param bufferRecordTickPeriod the period in number of ticks that data should be stored in the
    *                               buffer. Default value {@value #DEFAULT_BUFFER_RECORD_TICK_PERIOD}.
    */
   public void submitBufferRecordTickPeriod(int bufferRecordTickPeriod)
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
    * <p>
    * This is a non-blocking operation and the change is applied immediately.
    * </p>
    * 
    * @param publishPeriod period in nanoseconds at which the buffer data is publish while in
    *                      {@link SessionMode#RUNNING} mode. Default value
    *                      {@value #DEFAULT_BUFFER_PUBLISH_PERIOD}.
    */
   public void submitDesiredBufferPublishPeriod(long publishPeriod)
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
    * This is a blocking operation and will return only when done.
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
      pendingCropBufferRequest.submitAndWait(cropBufferRequest);
   }

   /**
    * Requests to fill in the buffer entirely or partially with zeros or the value stored in each
    * {@link YoVariable}.
    * <p>
    * This is a blocking operation and will return only when done.
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
      pendingFillBufferRequest.submitAndWait(fillBufferRequest);
   }

   /**
    * Requests to change the size the of the buffer.
    * <p>
    * This is typically used to increased the buffer size. To decrease the buffer size, it is
    * recommended to use a crop request that provides better control on the data being preserved.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done.
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
      pendingBufferSizeRequest.submitAndWait(bufferSizeRequest);
   }

   /**
    * Requests to move the current buffer index, i.e. reading/writing position.
    * <p>
    * This is a blocking operation and will return only when done.
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
      pendingBufferIndexRequest.submitAndWait(bufferIndexRequest);
   }

   /**
    * Requests to increment the current buffer index, i.e. reading/writing position, by a given step
    * size.
    * <p>
    * This is a blocking operation and will return only when done.
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
      pendingIncrementBufferIndexRequest.submitAndWait(incrementBufferIndexRequest);
   }

   /**
    * Requests to decrement the current buffer index, i.e. reading/writing position, by a given step
    * size.
    * <p>
    * This is a blocking operation and will return only when done.
    * </p>
    * <p>
    * This request is only processed if the session is in {@link SessionMode#PAUSE}, it will be ignored
    * otherwise.
    * </p>
    * 
    * @param decrementBufferIndexRequest the decrement to apply to the current buffer index.
    */
   public void submitDecrementBufferIndexRequestAndWait(Integer incrementBufferIndexRequest)
   {
      pendingDecrementBufferIndexRequest.submitAndWait(incrementBufferIndexRequest);
   }

   /**
    * Requests to set the buffer in-point, i.e. the first index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done.
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
      pendingBufferInPointIndexRequest.submitAndWait(bufferInPointIndexRequest);
   }

   /**
    * Requests to set the buffer out-point, i.e. the last index of the active part of the buffer.
    * <p>
    * The active part of the buffer is typically the sub-section that contains actual data. The active
    * part of the buffer is delimited by an in-point index and an out-point index.
    * </p>
    * <p>
    * This is a blocking operation and will return only when done.
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
      pendingBufferOutPointIndexRequest.submitAndWait(bufferOutPointIndexRequest);
   }

   /**
    * Requests to export this session's data to file.
    * <p>
    * This is a blocking operation and will return only when done.
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
      pendingDataExportRequest.submitAndWait(sessionDataExportRequest);
   }

   public void setSessionState(SessionState state)
   {
      if (state == SessionState.ACTIVE)
      {
         if (!sessionStarted)
         {
            LogTools.info("Starting session");
            startSessionThread();
            sessionStateChangedListeners.forEach(listener -> listener.accept(state));
         }
      }
      else
      {
         if (!isSessionShutdown)
         {
            shutdownSession();
            sessionStateChangedListeners.forEach(listener -> listener.accept(state));
         }
      }
   }

   public void startSessionThread()
   {
      LogTools.info("Started session's thread");
      sessionStarted = true;
      scheduleSessionTask(getActiveMode());
   }

   public void shutdownSession()
   {
      if (isSessionShutdown)
         return;

      isSessionShutdown = true;

      shutdownListeners.forEach(Runnable::run);

      LogTools.info("Stopped session's thread");
      sessionStarted = false;

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
      if (!sessionStarted)
         return;

      SessionMode previousMode = activeMode.get();

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

   protected void schedulingSessionMode(SessionMode previousMode, SessionMode newMode)
   {

   }

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

   protected void setSessionModeTask(SessionMode sessionMode, Runnable runnable)
   {
      sessionModeToTaskMap.put(sessionMode, runnable);
   }

   private void reportActiveMode()
   {
      for (Consumer<SessionProperties> listener : sessionPropertiesListeners)
         listener.accept(getSessionProperties());
   }

   public SessionProperties getSessionProperties()
   {
      return new SessionProperties(activeMode.get(),
                                   runAtRealTimeRate.get(),
                                   playbackRealTimeRate.get().doubleValue(),
                                   sessionDTNanoseconds.get(),
                                   bufferRecordTickPeriod.get());
   }

   /**
    * Called when starting this session regardless of the initial mode.
    */
   public void initializeSession()
   {
   }

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
         // Not sure why we wouldn't want that when starting in RUNNING.
         // When running simulation, the session starts in PAUSE, writing in the buffer allows to write the robot initial state.
         if (currentMode == SessionMode.PAUSE || currentMode == SessionMode.PLAYBACK)
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

   protected int nextRunBufferRecordTickCounter = 0;

   public void runTick()
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
   }

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
    * Performs action specific to the implementation of this session.
    *
    * @return the current time in seconds.
    */
   protected abstract double doSpecificRunTick();

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

   protected void initializePlaybackTick()
   {
      sharedBuffer.flushLinkedPushRequests();
      sharedBuffer.readBuffer();
   }

   protected void doSpecificPlaybackTick()
   {

   }

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

   protected long computePauseTaskPeriod()
   {
      return Conversions.secondsToNanoseconds(0.01);
   }

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

   protected boolean doSpecificPauseTick()
   {
      return false;
   }

   protected void finalizePauseTick(boolean shouldReadBuffer)
   {
      if (shouldReadBuffer)
         sharedBuffer.readBuffer();
      sharedBuffer.prepareLinkedBuffersForPull();
      publishBufferProperties(sharedBuffer.getProperties());
   }

   protected void publishBufferProperties(YoBufferPropertiesReadOnly bufferProperties)
   {
      for (Consumer<YoBufferPropertiesReadOnly> listener : currentBufferPropertiesListeners)
      {
         listener.accept(bufferProperties.copy());
      }
   }

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

   public YoDouble getTime()
   {
      return time;
   }

   public YoRegistry getRootRegistry()
   {
      return rootRegistry;
   }

   public boolean hasSessionStarted()
   {
      return sessionStarted;
   }

   public SessionMode getActiveMode()
   {
      return activeMode.get();
   }

   public boolean getRunAtRealTimeRate()
   {
      return runAtRealTimeRate.get();
   }

   public double getPlaybackRealTimeRate()
   {
      return playbackRealTimeRate.get().doubleValue();
   }

   public int getBufferRecordTickPeriod()
   {
      return bufferRecordTickPeriod.get();
   }

   public double getBufferRecordTimePeriod()
   {
      return getBufferRecordTickPeriod() * getSessionDTSeconds();
   }

   public double getSessionDTSeconds()
   {
      return sessionDTNanoseconds.get() * 1.0e-9;
   }

   public long getSessionDTNanoseconds()
   {
      return sessionDTNanoseconds.get();
   }

   public long getDesiredBufferPublishPeriod()
   {
      return desiredBufferPublishPeriod.get();
   }

   public abstract String getSessionName();

   public final ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   public abstract List<RobotDefinition> getRobotDefinitions();

   public abstract List<TerrainObjectDefinition> getTerrainObjectDefinitions();

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

   YoSharedBuffer getBuffer()
   {
      return sharedBuffer;
   }

   public LinkedYoVariableFactory getLinkedYoVariableFactory()
   {
      return sharedBuffer;
   }

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

      private final TopicListener<SessionState> sessionCurrentStateListener = Session.this::setSessionState;
      private final TopicListener<SessionMode> sessionCurrentModeListener = Session.this::setSessionMode;
      private final TopicListener<Long> sessionDTNanosecondsListener = Session.this::setSessionDTNanoseconds;
      private final TopicListener<Boolean> runAtRealTimeRateListener = Session.this::submitRunAtRealTimeRate;
      private final TopicListener<Double> playbackRealTimeRateListener = Session.this::submitPlaybackRealTimeRate;
      private final TopicListener<Integer> bufferRecordTickPeriodListener = Session.this::submitBufferRecordTickPeriod;
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

   public interface SessionModeTransition
   {
      SessionMode getNextMode();

      boolean isDone();

      default void notifyTransitionComplete()
      {
      }

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

   public interface SessionModeChangeListener
   {
      void onChange(SessionMode previousMode, SessionMode newMode);
   }

   public static class PeriodicTaskWrapper implements Runnable
   {
      private static final int ONE_MILLION = 1000000;

      private final Runnable task;
      private final long periodInNanos;
      private final AtomicBoolean running = new AtomicBoolean(true);
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
