package us.ihmc.scs2.session.remote;

import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionProperties;
import us.ihmc.scs2.session.tools.RobotModelLoader;
import us.ihmc.scs2.simulation.robot.Robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.stream.Collectors;

public class RemoteSession extends Session
{
   private static final double MAX_DELAY_MILLI = 200.0;

   private YoVariableClientInterface yoVariableClientInterface;

   private final String sessionName;
   private final List<Robot> robots = new ArrayList<>();
   private final List<RobotDefinition> robotDefinitions = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private final Runnable robotStateUpdater;

   private final AtomicLong serverTimestamp = new AtomicLong(-1);
   private final AtomicLong latestDataTimestamp = new AtomicLong(-1);
   private final LoggerStatusUpdater loggerStatusUpdater = new LoggerStatusUpdater();

   private int bufferRecordTickPeriod = 1;
   private boolean initializeServerUpdateRate = true;

   public RemoteSession(YoVariableClientInterface yoVariableClientInterface,
                        LogHandshake handshake,
                        YoVariableHandshakeParser handshakeParser,
                        DebugRegistry debugRegistry)
   {
      super();

      this.yoVariableClientInterface = yoVariableClientInterface;

      sessionName = yoVariableClientInterface.getServerName();

      rootRegistry.addChild(handshakeParser.getRootRegistry());
      rootRegistry.addChild(debugRegistry.getYoRegistry());
      yoGraphicDefinitions.add(new YoGraphicGroupDefinition("SCS1 YoGraphics",
                                                            YoGraphicConversionTools.toYoGraphicDefinitions(handshakeParser.getSCS1YoGraphics())));
      yoGraphicDefinitions.addAll(handshakeParser.getSCS2YoGraphics());

      RobotDefinition robotDefinition = RobotModelLoader.loadModel(handshake.getModelName(),
                                                                   handshake.getModelLoaderClass(),
                                                                   handshake.getResourceDirectories(),
                                                                   handshake.getModels(),
                                                                   handshake.getResourceZip());
      if (robotDefinition != null)
      {
         robotDefinitions.add(robotDefinition);
         Robot robot = new Robot(robotDefinition, getInertialFrame());
         robots.add(robot);
         robotStateUpdater = RobotModelLoader.setupRobotUpdater(robot, handshakeParser, rootRegistry);
      }
      else
      {
         robotStateUpdater = null;
      }

      setSessionMode(SessionMode.RUNNING);
      setSessionDTSeconds(handshakeParser.getDt());
      setSessionModeTask(SessionMode.RUNNING, () ->
      {
         if (!this.yoVariableClientInterface.isConnected())
            setSessionMode(SessionMode.PAUSE);
         /* Do nothing, the client thread calls runTick(). */
      });
      addSessionPropertiesListener(properties ->
                                   {
                                      if (properties.getActiveMode() == SessionMode.RUNNING)
                                         reconnect();
                                      else
                                         disconnect();
                                   });
      setDesiredBufferPublishPeriod(Conversions.secondsToNanoseconds(1.0 / 60.0));
   }

   public long getDelay()
   {
      return serverTimestamp.get() - latestDataTimestamp.get();
   }

   @Override
   protected long computeRunTaskPeriod()
   {
      return Conversions.secondsToNanoseconds(0.01);
   }

   @Override
   protected long computePlaybackTaskPeriod()
   {
      /*
       * We let the yoVariableClient handle the bufferRecordTickPeriod feature, so we're not setting
       * updating the corresponding field in Session. Thus, it cannot compute the playback DT properly
       * without temporarily setting the Session.bufferRecordTickPeriod.
       */
      int superBufferRecordTickPeriod = super.getBufferRecordTickPeriod();
      super.setBufferRecordTickPeriod(bufferRecordTickPeriod);
      long playbackTaskPeriod = super.computePlaybackTaskPeriod();
      super.setBufferRecordTickPeriod(superBufferRecordTickPeriod);
      return playbackTaskPeriod;
   }

   @Override
   public SessionProperties getSessionProperties()
   {
      return new SessionProperties(getActiveMode(),
                                   getRunAtRealTimeRate(),
                                   getPlaybackRealTimeRate(),
                                   getSessionDTNanoseconds(),
                                   bufferRecordTickPeriod,
                                   getRunMaxDuration());
   }

   public void receivedTimestampOnly(long timestamp)
   {
      serverTimestamp.set(timestamp);
   }

   public void receivedTimestampAndData(long timestamp)
   {
      if (!hasSessionStarted() || getActiveMode() != SessionMode.RUNNING)
         return;

      if (initializeServerUpdateRate)
      {
         updateServerUpdateRate();
         initializeServerUpdateRate = false;
         return;
      }

      latestDataTimestamp.set(timestamp);
      runTick();
   }

   private void reconnect()
   {
      if (yoVariableClientInterface.isConnected())
         return;

      sharedBuffer.setInPoint(sharedBuffer.getProperties().getCurrentIndex());

      try
      {
         if (yoVariableClientInterface.reconnect())
            updateServerUpdateRate();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void disconnect()
   {
      if (!yoVariableClientInterface.isConnected())
         return;

      initializeServerUpdateRate = true;
      yoVariableClientInterface.disconnect();
   }

   @Override
   protected double doSpecificRunTick()
   {
      if (robotStateUpdater != null)
         robotStateUpdater.run();
      return Conversions.nanosecondsToSeconds(latestDataTimestamp.get());
   }

   @Override
   protected void initializeRunTick()
   {
      if (firstRunTick)
      {
         sharedBuffer.incrementBufferIndex(true);
         sharedBuffer.setInPoint(sharedBuffer.getProperties().getCurrentIndex());
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

   @Override
   protected void finalizeRunTick(boolean forceWriteBuffer)
   {
      if (forceWriteBuffer || Conversions.nanosecondsToMilliseconds(getDelay()) < MAX_DELAY_MILLI * bufferRecordTickPeriod)
      {
         super.finalizeRunTick(forceWriteBuffer);
      }
      else
      {
         sharedBuffer.writeBuffer();
         processBufferRequests(false);
         publishBufferProperties(sharedBuffer.getProperties());
      }
   }

   public void receivedCommand(DataServerCommand command, int argument)
   {
      loggerStatusUpdater.updateStatus(command, argument);
   }

   @Override
   public void setBufferRecordTickPeriod(int bufferRecordTickPeriod)
   {
      if (bufferRecordTickPeriod == this.bufferRecordTickPeriod)
         return;
      this.bufferRecordTickPeriod = Math.max(1, bufferRecordTickPeriod);
      updateServerUpdateRate();
   }

   private void updateServerUpdateRate()
   {
      int updateRateInMilliseconds = (int) TimeUnit.NANOSECONDS.toMillis(bufferRecordTickPeriod * getSessionDTNanoseconds());
      yoVariableClientInterface.setVariableUpdateRate(updateRateInMilliseconds);
   }

   public void close()
   {
      if (yoVariableClientInterface != null)
      {
         if (yoVariableClientInterface.isConnected())
            yoVariableClientInterface.disconnect();
         yoVariableClientInterface.stop();
      }
   }

   public void sendCommandToYoVariableServer(DataServerCommand command, int argument)
   {
      if (yoVariableClientInterface != null && yoVariableClientInterface.isConnected())
         yoVariableClientInterface.sendCommand(command, argument);
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
   public List<RobotStateDefinition> getCurrentRobotStateDefinitions(boolean initialState)
   {
      return robots.stream().map(Robot::getCurrentRobotStateDefinition).collect(Collectors.toList());
   }

   public LoggerStatusUpdater getLoggerStatusUpdater()
   {
      return loggerStatusUpdater;
   }
}
