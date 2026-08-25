package us.ihmc.scs2.session.log;

import logger_msgs.LogProperties;
import logger_msgs.Synchronization;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionRobotDefinitionListChange;
import us.ihmc.scs2.session.tools.RobotDataLogTools;
import us.ihmc.scs2.session.tools.RobotModelLoader;
import us.ihmc.scs2.sharedMemory.HistoricalValueBitsSource;
import us.ihmc.scs2.sharedMemory.HistoricalValueBitsWriter;
import us.ihmc.scs2.sharedMemory.YoIntegerBuffer;
import us.ihmc.scs2.sharedMemory.YoVariableBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

public class LogSession extends Session
{
   private final String sessionName;
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private Runnable robotStateUpdater;
   private RobotDefinition robotDefinition;
   private Robot robot;

   private final File logDirectory;
   private final CompositeLogDataReader logDataReader;
   private final LogPropertiesReader logProperties;

   /**
    * This flag is changed to true whenever the session has been initialized. If a log is added that overrides the robotStateUpdater,
    * this flag tells the updater whether to run.
    */
   private boolean initialized = false;

   /**
    * This is used to jump to a specific position in the log when the user drags the slider.
    * <p>
    * It is thread-safe.
    * </p>
    */
   private final AtomicInteger logPositionRequest = new AtomicInteger(-1);

   private final List<TimeConsumer> afterReadCallbacks = new ArrayList<>();
   private final List<Consumer<List<YoGraphicDefinition>>> graphicsAddedCallbacks = new ArrayList<>();

   /**
    * Per-log-registry allow-list of variables that should get a buffer allocated immediately (those referenced by that
    * log's own {@link YoGraphicDefinition}s), keyed by that log's root registry. Everything else under a known key gets
    * its buffer created lazily, on first use (e.g. opening a chart for it) - see {@link #isEagerLogVariable}. A
    * variable whose registry isn't a descendant of any key here (robot joints, session/user/equation registries, etc.)
    * is always eager; this only restricts the raw per-log variable trees, which is where nearly all of a log's
    * variable count lives and where almost none of it is ever looked at in a given session.
    */
   /**
    * Keyed by the {@link YoVariable} object itself, not its full name: {@link YoVariable#getFullNameString()} is
    * lazily cached (see {@link YoVariable#resetFullName()}), and {@link YoRegistry#addChild} - which every one of
    * these variables goes through right after this map is populated, when its log's registry tree gets attached to
    * {@link #rootRegistry} - recursively invalidates that cache for the whole attached subtree. A name captured here
    * beforehand and a name queried afterward are therefore never equal (the latter gains the attached tree's prefix),
    * so identity is the only key that stays valid across that attachment.
    */
   private final Map<YoRegistry, Set<YoVariable>> lazyBufferScopeEagerVariables = new HashMap<>();
   private boolean lazyBufferFilterInstalled = false;

   /**
    * Which {@link LogDataReader} owns a given lazy variable, and its index within that reader's own
    * {@link LogDataReader#getYoVariablesList()} - built once per log in {@link #configureLazyBufferPolicy} and
    * consulted by {@link #getHistoricalValueBits} to fetch a value directly from that log's file on demand. Keyed by
    * identity for the same reason as {@link #lazyBufferScopeEagerVariables}.
    */
   private final Map<YoVariable, LazyLogVariableIndex> lazyLogVariableIndexByVariable = new HashMap<>();

   /**
    * One dedicated {@link LogRandomAccessValueReader} per {@link LogDataReader} (main log and each child log), opened
    * once in {@link #configureLazyBufferPolicy} and reused by every {@link #getHistoricalValueBits} call for that log
    * - see {@link LogRandomAccessValueReader}'s class doc for why this needs to stay separate from that
    * {@link LogDataReader}'s own live-playback channel/cursor rather than sharing it.
    */
   private final Map<LogDataReader, LogRandomAccessValueReader> randomAccessValueReaders = new HashMap<>();

   private record LazyLogVariableIndex(LogDataReader reader, int variableIndex)
   {
   }

   public LogSession(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.logDirectory = logDirectory;
      try
      {
         logDataReader = new CompositeLogDataReader(logDirectory, progressConsumer);
         LogTools.info("Created data reader.");
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      logProperties = logDataReader.getLogProperties();
      sessionName = logProperties.getNameAsString();

      addLogToDataContainers(logDataReader, true);
      addLogInternal(logDataReader, false);

      for (int i = 0; i < logProperties.getChildLogs().size(); i++)
      {
         LogDataReader childLog = logDataReader.getChildLogDataReaders().get(i);
         addLogToDataContainers(childLog, false);
         addLogInternal(childLog, false);
      }

      setDesiredBufferPublishPeriod(Conversions.secondsToNanoseconds(1.0 / 30.0));
      setSessionDTSeconds(logDataReader.getDt());
      setSessionMode(SessionMode.PAUSE);
   }

   private void addLogToDataContainers(LogDataReader logDataReader, boolean isMain) throws IOException
   {
      configureLazyBufferPolicy(logDataReader);

      rootRegistry.addChild(logDataReader.getLocalYoRegistry());
      if (isMain)
         rootRegistry.addChild(logDataReader.getLogRootRegistry());
      else
      {
         // We're instituting a 1-step down registry here so that there are registry name conflicts.
         YoRegistry addedLog = new YoRegistry(logDataReader.getLogDirectory().getName());
         rootRegistry.addChild(addedLog);
         addedLog.addChild(logDataReader.getLogRootRegistry());
      }

      List<YoGraphicDefinition> addedGraphics = new ArrayList<>();
      // update the graphic definitions.
      if (logDataReader.getLogSCS2YoGraphics() != null && !logDataReader.getLogSCS2YoGraphics().isEmpty())
      {
         logDataReader.getLogSCS2YoGraphics().forEach( graphics ->
               {
                     if (!graphics.isEmpty())
                        addedGraphics.add(graphics);
               });
      }
      yoGraphicDefinitions.addAll(addedGraphics);


      // This alerts the graphics system that it needs to update. if we've added a log to an existing session, its graphics need to be loaded by the
      // YoGraphicFXManager
      graphicsAddedCallbacks.forEach(consumer -> consumer.accept(addedGraphics));
   }

   /**
    * Restricts eager buffer allocation to the variables {@code logToAdd} actually needs rendered immediately (those
    * referenced by its own {@link YoGraphicDefinition}s), instead of every one of its variables. This log's raw
    * variable count can be in the tens of thousands while the variables actually driving a graphic are typically a
    * handful, so this is where practically all of the savings comes from. Called before every place that attaches a
    * log's registry tree to {@link #rootRegistry}, so it covers the main log, child logs added at construction, and
    * ones added later via {@link #addLogAtDirectory}.
    * <p>
    * Robot joint variables aren't affected - those live in a separate registry ({@code robot.getRegistry()}, added in
    * {@link RobotModelLoader#setupRobotUpdater}) that this method never scopes, so they remain eager as before.
    * </p>
    */
   private void configureLazyBufferPolicy(LogDataReader logToAdd) throws IOException
   {
      lazyBufferScopeEagerVariables.put(logToAdd.getLogRootRegistry(), computeYoGraphicReferencedVariables(logToAdd));

      List<YoVariable> logVariables = logToAdd.getYoVariablesList();
      for (int i = 0; i < logVariables.size(); i++)
         lazyLogVariableIndexByVariable.put(logVariables.get(i), new LazyLogVariableIndex(logToAdd, i));

      randomAccessValueReaders.put(logToAdd, logToAdd.openRandomAccessValueReader());

      if (!lazyBufferFilterInstalled)
      {
         lazyBufferFilterInstalled = true;
         sharedBuffer.setEagerVariableFilter(this::isEagerLogVariable);
         sharedBuffer.setOnDemandBufferCreatedListener(buffer -> buffer.setHistoricalValueBitsSource(this::getHistoricalValueBits));
      }
   }

   /**
    * Fetches {@code variable}'s value for every index in {@code [from, from + length)} straight from whichever log it
    * belongs to, instead of requiring the log to have already been replayed that far - installed as every
    * lazily-created buffer's {@link HistoricalValueBitsSource} in {@link #configureLazyBufferPolicy}. Only ever
    * consulted for a range a chart (or similar consumer) actually asked for - see {@link YoVariableBuffer#copy} - so
    * the cost scales with what was actually looked at rather than with how far the log has played.
    * <p>
    * Reads through {@link #randomAccessValueReaders}, a dedicated {@link LogRandomAccessValueReader} per log with its
    * own channel and batch cache, entirely separate from that log's live-playback {@link LogDataReader}. That
    * separation (rather than reusing the live reader's channel with a save-position/restore-position dance around
    * each call) is what lets repeatedly re-visiting the same region - e.g. scrubbing a chart back and forth - reuse
    * an already-decompressed batch instead of reloading it: restoring a shared channel's live position after each
    * call would otherwise land it in whatever batch playback happens to be at, evicting the one this range just
    * decompressed before the very next call (e.g. the next redraw of the same scrub) could reuse it.
    * </p>
    * <p>
    * A buffer index is <em>not</em> a log position, so each one is translated through
    * {@link #logPositionAtBufferIndex(int)} first. An index the main log never recorded into has no value to fetch,
    * and is written as {@code 0} rather than left alone - leaving it unwritten would leave it unpopulated, and
    * {@link YoVariableBuffer} would re-request it (and pay the seek) on every subsequent read forever. Playback
    * writing genuine data there later still overwrites it through the ordinary
    * {@link YoVariableBuffer#writeBufferAt} path.
    * </p>
    */
   private void getHistoricalValueBits(YoVariable variable, int from, int length, HistoricalValueBitsWriter writer)
   {
      LazyLogVariableIndex source = lazyLogVariableIndexByVariable.get(variable);
      if (source == null)
      {
         long bits = variable.getValueAsLongBits();
         for (int i = 0; i < length; i++)
            writer.write(from + i, bits);
         return;
      }

      LogDataReader reader = source.reader();
      LogRandomAccessValueReader randomAccessReader = randomAccessValueReaders.get(reader);
      ChildLogSynchronization synchronization = reader == logDataReader ? null : findSynchronization(reader);

      // Resolved once per call, not per index: this is the live array, and only the session thread - the one running
      // this - ever resizes it, so it cannot be swapped out from under this loop.
      int[] bufferIndexToLogPosition = bufferIndexToLogPosition();
      int mainLogEntryCount = logDataReader.getNumberOfEntries();

      for (int i = 0; i < length; i++)
      {
         int index = from + i;

         // Stored as position + 1, so 0 (an index never written) becomes -1.
         int mainLogPosition = bufferIndexToLogPosition == null || index < 0 || index >= bufferIndexToLogPosition.length ?
               -1 :
               bufferIndexToLogPosition[index] - 1;

         if (mainLogPosition < 0 || mainLogPosition >= mainLogEntryCount)
         { // This buffer index holds no log data at all - never written, or written before a crop shifted it away.
            writer.write(index, 0L);
            continue;
         }

         int localPosition = mainLogPosition;

         if (reader != logDataReader)
         { // computeChildPosition maps a *parent log position* onto this child's own timeline, not a buffer index.
            long relativePosition = synchronization == null ? mainLogPosition : synchronization.computeChildPosition(mainLogPosition);
            if (relativePosition < 0 || relativePosition >= reader.getNumberOfEntries())
            {
               writer.write(index, 0L); // Outside this child log's recorded range (e.g. it hadn't started yet).
               continue;
            }
            localPosition = (int) relativePosition;
         }

         writer.write(index, randomAccessReader.readVariableValueBitsAt(localPosition, source.variableIndex()));
      }
   }

   /**
    * The per-buffer-index record of which main-log tick was read into that index, as {@code position + 1} (so a
    * never-written index reads as {@code 0}), or {@code null} if that record isn't available.
    * <p>
    * A buffer index and a log position are not interchangeable: they only coincide for a log played straight through
    * from tick 0, with a buffer record period of 1, that has not yet wrapped the ring buffer and has never been
    * scrubbed, cropped or resized. Every one of those breaks the correspondence, and the buffer outlives all of them,
    * so the mapping has to be read back out of the buffer rather than assumed.
    * </p>
    * <p>
    * {@link LogDataReader#getCurrentRecordTickVariable()} is exactly that record - it is buffered eagerly alongside
    * everything else, and gets shuffled by a crop/resize in lockstep with the data it describes. See that method for
    * the off-by-one.
    * </p>
    */
   private int[] bufferIndexToLogPosition()
   {
      // Re-fetched rather than cached: YoIntegerBuffer#getBuffer hands out the live array, which resizeBuffer
      // replaces wholesale.
      YoVariableBuffer<?> recordTickBuffer = sharedBuffer.getRegistryBuffer()
                                                         .findYoVariableBuffer(logDataReader.getCurrentRecordTickVariable());

      return recordTickBuffer instanceof YoIntegerBuffer yoIntegerBuffer ? yoIntegerBuffer.getBuffer() : null;
   }

   private ChildLogSynchronization findSynchronization(LogDataReader reader)
   {
      for (ChildLogData childLogData : logDataReader.getChildLogData())
         if (childLogData.getChildLogDataReader() == reader)
            return childLogData.getSynchronization();
      return null;
   }

   /**
    * @return {@code true} if {@code variable} isn't under any log's raw variable tree (always eager - robot joints,
    *       session/user/equation registries, etc.), or if it is and was found to be referenced by that log's own
    *       {@link YoGraphicDefinition}s.
    */
   private boolean isEagerLogVariable(YoVariable variable)
   {
      YoRegistry registry = variable.getRegistry();

      for (Map.Entry<YoRegistry, Set<YoVariable>> scope : lazyBufferScopeEagerVariables.entrySet())
      {
         if (isDescendantOrSelf(registry, scope.getKey()))
            return scope.getValue().contains(variable);
      }

      return true;
   }

   private static boolean isDescendantOrSelf(YoRegistry candidate, YoRegistry ancestor)
   {
      for (YoRegistry current = candidate; current != null; current = current.getParent())
      {
         if (current == ancestor)
            return true;
      }
      return false;
   }

   /**
    * Matches every field value of every {@link YoGraphicDefinition} under {@code logToAdd}'s graphics against
    * {@code logToAdd}'s own variables, preferring an exact full-name match and falling back to an unambiguous
    * short-name match. A field that matches nothing (a constant, a color, an ambiguous short name, or a reference
    * outside this log) is simply not included - worst case that graphic's binding stays unbuffered until something
    * else (e.g. a chart) asks for it, same as it would if this method didn't run at all.
    * <p>
    * Matching itself still goes through {@link YoVariable#getFullNameString()} - at this point, before {@code
    * logToAdd}'s registry tree gets attached to {@link #rootRegistry}, that reflects the log's own internal path,
    * which is what a {@link YoGraphicDefinition} field value (recorded by the log, independent of any session) refers
    * to. The returned set holds the matched variables themselves rather than those (soon stale) names, so callers can
    * check membership by identity instead - see {@link #lazyBufferScopeEagerVariables}.
    * </p>
    */
   private static Set<YoVariable> computeYoGraphicReferencedVariables(LogDataReader logToAdd)
   {
      List<YoGraphicGroupDefinition> graphics = logToAdd.getLogSCS2YoGraphics();
      if (graphics == null || graphics.isEmpty())
         return Collections.emptySet();

      Map<String, YoVariable> fullNameToVariable = new HashMap<>();
      Map<String, List<YoVariable>> shortNameToVariables = new HashMap<>();
      for (YoVariable variable : logToAdd.getYoVariablesList())
      {
         fullNameToVariable.put(variable.getFullNameString(), variable);
         shortNameToVariables.computeIfAbsent(variable.getName(), name -> new ArrayList<>()).add(variable);
      }

      Set<YoVariable> eagerVariables = new HashSet<>();

      for (YoGraphicGroupDefinition group : graphics)
      {
         for (YoGraphicDefinition.YoGraphicFieldsSummary summary : YoGraphicDefinition.exportSubtreeYoGraphicFieldsSummaryList(group))
         {
            for (YoGraphicDefinition.YoGraphicFieldInfo field : summary)
            {
               String value = field.getFieldValue();
               if (value == null || value.isEmpty())
                  continue;

               YoVariable exactMatch = fullNameToVariable.get(value);
               if (exactMatch != null)
               {
                  eagerVariables.add(exactMatch);
                  continue;
               }

               String shortName = value.contains(".") ? value.substring(value.lastIndexOf('.') + 1) : value;
               List<YoVariable> candidates = shortNameToVariables.get(shortName);
               if (candidates != null && candidates.size() == 1)
                  eagerVariables.add(candidates.get(0));
            }
         }
      }

      return eagerVariables;
   }

   public void bindSynchronization(String logToSynchronize, String mainLogVarName, String logToSyncVarName)
   {
      Synchronization synchronization = logDataReader.synchronizeChildLogWithParent(logToSynchronize, mainLogVarName, logToSyncVarName);
      // We want to set the offset to 0, so that whenever this is exported, everything starts from the same point
      synchronization.setOffset(0);
      int childNumber = logDataReader.getLogNumber(logToSynchronize);
      logDataReader.getLogProperties().getChildLogs().get(childNumber).getSynchronization().set(synchronization);
   }

   public ChildLogData addLogAtDirectory(File logDirectory) throws IOException
   {
      ChildLogData addedDataReader = logDataReader.addChildLog(logDirectory);
      if (addedDataReader != null)
      {
         addLogToDataContainers(addedDataReader.getChildLogDataReader(), false);
         addLogInternal(addedDataReader.getChildLogDataReader(), true);
      }
      return addedDataReader;
   }

   private void addLogInternal(LogDataReader logToAdd, boolean notifyListeners) throws IOException
   {
      if (robotStateUpdater == null)
      {
         File logDirectory = logToAdd.getLogDirectory();
         LogProperties logProperties = logToAdd.getLogProperties();
         RobotDefinition robotDefinition = RobotDataLogTools.loadRobotDefinition(logDirectory, logProperties);

         if (robotDefinition != null)
         {
            this.robotDefinition = robotDefinition;
            robot = new Robot(robotDefinition, getInertialFrame());
            robotStateUpdater = RobotModelLoader.setupRobotUpdater(robot, logToAdd.getJointStates(), rootRegistry);
            if (initialized)
               robotStateUpdater.run();

            if (notifyListeners)
            {
               SessionRobotDefinitionListChange change = SessionRobotDefinitionListChange.add(robotDefinition);
               reportRobotDefinitionListChange(change);
            }
         }
         else
         {
            robotStateUpdater = null;
            robot = null;
            this.robotDefinition = null;
         }
      }
   }

   @Override
   public void addGraphicsAddedCallback(Consumer<List<YoGraphicDefinition>> graphicsChangedCallback)
   {
      graphicsAddedCallbacks.add(graphicsChangedCallback);
   }


   public void submitLogPositionRequest(int logPosition)
   {
      logPositionRequest.set(logPosition);
   }

   @Override
   protected void initializeSession()
   {
      // We read the very first frame of the log.
      logDataReader.read();

      if (robotStateUpdater != null)
         robotStateUpdater.run();
      initialized = true;
   }

   @Override
   protected void initializeRunTick()
   {
      if (firstRunTick)
      {
         // TODO Can probably be a little smarter here, sometimes we don't need to reset the equation manager.
         equationManager.reset();

         YoBufferPropertiesReadOnly properties = sharedBuffer.getProperties();

         if (properties.getCurrentIndex() != properties.getOutPoint())
            sharedBuffer.setInPoint(properties.getCurrentIndex());
         else if (!firstLogPositionRequest) // That means the user has scrubbed through the data.
            sharedBuffer.setInPoint(properties.getCurrentIndex());
         sharedBuffer.incrementBufferIndex(true);
         // Sync the log position index (logDataReader.index) the current YoVariable (logDataReader.currentRecordTick()) value.
         // Without that, scrubbing through a chart and then resuming log reading will start from an arbitrary position in the log file (corresponding to where we last stop reading the log file).
         logDataReader.seek(logDataReader.getCurrentLogPosition());
         nextRunBufferRecordTickCounter = 0;
         runTickCounter = 0L;
         firstRunTick = false;
      }
      else if (nextRunBufferRecordTickCounter <= 0)
      {
         sharedBuffer.incrementBufferIndex(true);
         sharedBuffer.processLinkedPushRequests(false);
      }

      // Push from the linked registries are unnecessary when reading a log file.
   }

   @Override
   protected double doSpecificRunTick()
   {
      boolean endOfLog = logDataReader.read();
      if (endOfLog)
         setSessionMode(SessionMode.PAUSE);

      if (robotStateUpdater != null)
         robotStateUpdater.run();

      double currentTime = logDataReader.getCurrentRobotTime();

      for (int i = 0; i < afterReadCallbacks.size(); i++)
         afterReadCallbacks.get(i).accept(currentTime);

      return currentTime;
   }

   private boolean firstLogPositionRequest = true;

   @Override
   public void pauseTick()
   {
      if (firstPauseTick)
         firstLogPositionRequest = true;

      int logPosition = logPositionRequest.getAndSet(-1);

      if (logPosition == -1)
      {
         super.pauseTick();
      }
      else
      {// Handles when the user is scrubbing through the log using the log slider.
         processBufferRequests(false);

         logDataReader.seek(logPosition);
         logDataReader.read();

         if (robotStateUpdater != null)
            robotStateUpdater.run();

         if (firstLogPositionRequest)
         { // We increment only once when starting to scrub through the data to not write on the last data point.
            sharedBuffer.incrementBufferIndex(true);
            firstLogPositionRequest = false;
         }
         sharedBuffer.writeBuffer();
         sharedBuffer.prepareLinkedBuffersForPull();
         publishBufferProperties(sharedBuffer.getProperties());
      }
   }

   /**
    * Adds a callback to be executed after reading each line of the log file.
    * <p>
    * This can be used to register some post-processing on the log data.
    * </p>
    *
    * @param callback the callback to add.
    */
   public void addAfterReadCallback(TimeConsumer callback)
   {
      afterReadCallbacks.add(Objects.requireNonNull(callback, "The callback cannot be null."));
   }

   /**
    * Removes a callback that was previously added.
    *
    * @param callback the callback to remove.
    * @return {@code true} if the callback was removed, {@code false} if it was not found.
    */
   public boolean removeAfterReadCallback(TimeConsumer callback)
   {
      return afterReadCallbacks.remove(callback);
   }

   @Override
   public String getSessionName()
   {
      return sessionName;
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      if (robotDefinition == null)
         return Collections.emptyList();
      return Collections.singletonList(robotDefinition);
   }

   @Override
   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return Collections.emptyList();
   }

   @Override
   public List<YoGraphicDefinition> getYoGraphicDefinitions()
   {
      return yoGraphicDefinitions;
   }

   public List<Robot> getRobots()
   {
      if (robot == null)
         return Collections.emptyList();
      return Collections.singletonList(robot);
   }

   @Override
   public List<RobotStateDefinition> getCurrentRobotStateDefinitions(boolean initialState)
   {
      if (robot == null)
         return Collections.emptyList();
      return Collections.singletonList(robot.getCurrentRobotStateDefinition());
   }

   public File getLogDirectory()
   {
      return logDirectory;
   }

   public CompositeLogDataReader getLogDataReader()
   {
      return logDataReader;
   }

   public LogPropertiesReader getLogProperties()
   {
      return logProperties;
   }
}
