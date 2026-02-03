package us.ihmc.scs2.session.log;

import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.Synchronization;
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
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
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

   private void addLogToDataContainers(LogDataReader logDataReader, boolean isMain)
   {
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
      addedGraphics.add(new YoGraphicGroupDefinition("SCS1 YoGraphics", YoGraphicConversionTools.toYoGraphicDefinitions(logDataReader.getLogSCS1YoGraphics())));

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

   public LogDataReader getLogDataReader()
   {
      return logDataReader;
   }

   public LogPropertiesReader getLogProperties()
   {
      return logProperties;
   }
}
