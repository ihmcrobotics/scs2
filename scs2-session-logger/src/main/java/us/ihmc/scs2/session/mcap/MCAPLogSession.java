package us.ihmc.scs2.session.mcap;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MCAPLogSession extends Session
{
   // FIXME Figure out how to name the session
   private final String sessionName = getClass().getSimpleName();
   private final List<Robot> robots = new ArrayList<>();
   private final List<RobotDefinition> robotDefinitions = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private final Runnable robotStateUpdater;
   private final MCAPLogFileReader mcapLogFileReader;

   private final YoRegistry mcapRegistry = new YoRegistry("MCAP");

   /**
    * This is used to jump to a specific position in the log when the user drags the slider.
    * <p>
    * It is thread-safe.
    * </p>
    */
   private final AtomicInteger logPositionRequest = new AtomicInteger(-1);

   public MCAPLogSession(File mcapFile, MCAPDebugPrinter printer) throws IOException
   {
      mcapLogFileReader = new MCAPLogFileReader(mcapFile, printer, getInertialFrame(), mcapRegistry);
      // FIXME Do we need this guy?
      robotStateUpdater = null;
      mcapLogFileReader.loadSchemas();
      mcapLogFileReader.loadChannels();
      mcapLogFileReader.printStatistics();
      yoGraphicDefinitions.add(mcapLogFileReader.getYoGraphic());

      rootRegistry.addChild(mcapRegistry);
   }

   public void submitLogPositionRequest(int logPosition)
   {
      logPositionRequest.set(logPosition);
   }

   @Override
   protected void initializeSession()
   {
      try
      {
         mcapLogFileReader.initialize();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   protected double doSpecificRunTick()
   {
      if (mcapLogFileReader.incrementTimestamp())
      {
         setSessionMode(SessionMode.PAUSE);
      }
      else
      {
         try
         {
            mcapLogFileReader.readMessagesAtCurrentTimestamp();
         }
         catch (IOException e)
         {
            setSessionMode(SessionMode.PAUSE);
            throw new RuntimeException(e);
         }
      }
      return mcapLogFileReader.getCurrentTimeInLog();
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

         mcapLogFileReader.setCurrentTimestamp(mcapLogFileReader.getChunkManager().getTimestampAtIndex(logPosition));
         try
         {
            mcapLogFileReader.readMessagesAtCurrentTimestamp();
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }

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

   public MCAPLogFileReader getMCAPLogFileReader()
   {
      return mcapLogFileReader;
   }

   public long getRelativeTimestampAtIndex(int index)
   {
      return mcapLogFileReader.getRelativeTimestampAtIndex(index);
   }

   public File getMCAPFile()
   {
      return mcapLogFileReader.getMcapFile();
   }
}
