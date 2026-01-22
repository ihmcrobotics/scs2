package us.ihmc.scs2.session.log;

import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.LogProperties;
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
import java.util.stream.Collectors;

public class LogSession extends Session
{
   private final String sessionName;
   private final List<Robot> robots = new ArrayList<>();
   private final List<RobotDefinition> robotDefinitions = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private Runnable robotStateUpdater;

   private final File logDirectory;
   private final CompositeLogDataReader logDataReader;
   private final LogProperties logProperties;

   /**
    * This is used to jump to a specific position in the log when the user drags the slider.
    * <p>
    * It is thread-safe.
    * </p>
    */
   private final AtomicInteger logPositionRequest = new AtomicInteger(-1);

   private final List<TimeConsumer> afterReadCallbacks = new ArrayList<>();
   private final List<Runnable> graphicsChangedCallbacks = new ArrayList<>();

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

      addLogToStructs(logDataReader, true);

      sessionName = logProperties.getNameAsString();

      RobotDefinition robotDefinition = RobotDataLogTools.loadRobotDefinition(logDirectory, logProperties);

      if (robotDefinition != null)
      {
         robotDefinitions.add(robotDefinition);
         Robot robot = new Robot(robotDefinition, getInertialFrame());
         robots.add(robot);
         // TODO add to the add Logs.
         robotStateUpdater = RobotModelLoader.setupRobotUpdater(robot, logDataReader.getJointStates(), rootRegistry);
      }
      else
      {
         robotStateUpdater = null;
      }

      setDesiredBufferPublishPeriod(Conversions.secondsToNanoseconds(1.0 / 30.0));
      setSessionDTSeconds(logDataReader.getDt());
      setSessionMode(SessionMode.PAUSE);
   }

   private void addLogToStructs(LogDataReaderInterface logDataReader, boolean isMain) throws IOException
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

      // update the graphic definitions.
      yoGraphicDefinitions.add(new YoGraphicGroupDefinition("SCS1 YoGraphics", YoGraphicConversionTools.toYoGraphicDefinitions(logDataReader.getLogSCS1YoGraphics())));

      if (logDataReader.getLogSCS2YoGraphics() != null && !logDataReader.getLogSCS2YoGraphics().isEmpty())
      {
         logDataReader.getLogSCS2YoGraphics().forEach( graphics ->
               {
                     if (!graphics.isEmpty())
                        yoGraphicDefinitions.add(graphics);
               });
      }


      // This alerts the graphics system that it needs to update. if we've added a log to an existing session, its graphics need to be loaded by the
      // YoGraphicFXManager
      graphicsChangedCallbacks.forEach(Runnable::run);
   }

   public void bindSynchronization(String logToSynchronize, String mainLogVarName, String logToSyncVarName)
   {
      logDataReader.bindSynchronization(logToSynchronize, mainLogVarName, logToSyncVarName);
   }

   public LogDataReaderInterface addLog(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      LogDataReaderInterface addedDataReader = logDataReader.addLog(logDirectory, progressConsumer);
      if (addedDataReader != null)
      {
         addLogToStructs(addedDataReader, false);

         if (robotStateUpdater == null)
         {
            LogProperties logProperties = addedDataReader.getLogProperties();
            RobotDefinition robotDefinition = RobotDataLogTools.loadRobotDefinition(logDirectory, logProperties);

            if (robotDefinition != null)
            {
               robotDefinitions.add(robotDefinition);
               Robot robot = new Robot(robotDefinition, getInertialFrame());
               robots.add(robot);
               robotStateUpdater = RobotModelLoader.setupRobotUpdater(robot, addedDataReader.getJointStates(), rootRegistry);
               robotStateUpdater.run();

               SessionRobotDefinitionListChange change = SessionRobotDefinitionListChange.add(robotDefinition);
               reportRobotDefinitionListChange(change);
            }
            else
            {
               robotStateUpdater = null;
            }
         }

      }
      return addedDataReader;
   }

   @Override
   public void addGraphicsChangedCallback(Runnable graphicsChangedCallback)
   {
      graphicsChangedCallbacks.add(graphicsChangedCallback);
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
      return robotDefinitions;
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
      return robots;
   }

   @Override
   public List<RobotStateDefinition> getCurrentRobotStateDefinitions(boolean initialState)
   {
      return robots.stream().map(Robot::getCurrentRobotStateDefinition).collect(Collectors.toList());
   }

   public File getLogDirectory()
   {
      return logDirectory;
   }

   public CompositeLogDataReader getLogDataReader()
   {
      return logDataReader;
   }

   public LogProperties getLogProperties()
   {
      return logProperties;
   }
}
