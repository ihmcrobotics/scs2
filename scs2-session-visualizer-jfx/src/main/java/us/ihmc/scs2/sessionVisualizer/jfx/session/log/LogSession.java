package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.RobotDataLogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.RobotModelLoader;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.SCS1GraphicConversionTools;
import us.ihmc.scs2.simulation.robot.Robot;

public class LogSession extends Session
{
   private final String sessionName;
   private final List<Robot> robots = new ArrayList<>();
   private final List<RobotDefinition> robotDefinitions = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions;
   private final Runnable robotStateUpdater;

   private final File logDirectory;
   private final LogDataReader logDataReader;
   private final LogPropertiesReader logProperties;

   private final AtomicInteger logPositionRequest = new AtomicInteger(-1);

   public LogSession(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.logDirectory = logDirectory;
      try
      {
         logDataReader = new LogDataReader(logDirectory, progressConsumer);
         LogTools.info("Created data reader.");
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      logProperties = logDataReader.getLogProperties();

      YoVariableHandshakeParser parser = logDataReader.getParser();
      rootRegistry.addChild(logDataReader.getYoRegistry());
      rootRegistry.addChild(parser.getRootRegistry());
      yoGraphicDefinitions = SCS1GraphicConversionTools.toYoGraphicDefinitions(parser.getYoGraphicsListRegistry());

      sessionName = logProperties.getNameAsString();

      RobotDefinition robotDefinition = RobotDataLogTools.loadRobotDefinition(logDirectory, logProperties);

      if (robotDefinition != null)
      {
         robotDefinitions.add(robotDefinition);
         Robot robot = new Robot(robotDefinition, getInertialFrame());
         robots.add(robot);
         robotStateUpdater = RobotModelLoader.setupRobotUpdater(robot, parser, rootRegistry);
      }
      else
      {
         robotStateUpdater = null;
      }

      submitDesiredBufferPublishPeriod(Conversions.secondsToNanoseconds(1.0 / 30.0));
      setSessionDTSeconds(parser.getDt());
      setSessionMode(SessionMode.PAUSE);
   }

   public void submitLogPositionRequest(int logPosition)
   {
      logPositionRequest.set(logPosition);
   }

   @Override
   protected void initializeRunTick()
   {
      if (firstRunTick)
      {
         if (sharedBuffer.getProperties().getCurrentIndex() != sharedBuffer.getProperties().getOutPoint())
            sharedBuffer.setInPoint(sharedBuffer.getProperties().getCurrentIndex());
         sharedBuffer.incrementBufferIndex(true);
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
      return logDataReader.getCurrentRobotTime();
   }

   @Override
   public void pauseTick()
   {
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

         sharedBuffer.incrementBufferIndex(true);
         sharedBuffer.writeBuffer();
         sharedBuffer.prepareLinkedBuffersForPull();
         publishBufferProperties(sharedBuffer.getProperties());
      }
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

   @Override
   public RobotStateDefinition getCurrentRobotStateDefinition(RobotDefinition robotDefinition)
   {
      int indexOf = robotDefinitions.indexOf(robotDefinition);
      if (indexOf == -1)
         return null;
      Robot robot = robots.get(indexOf);
      return extractRobotState(robot.getName(), robot.getRootBody());
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
