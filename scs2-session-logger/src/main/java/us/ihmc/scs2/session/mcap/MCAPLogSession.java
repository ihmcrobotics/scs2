package us.ihmc.scs2.session.mcap;

import org.apache.commons.io.FilenameUtils;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionRobotDefinitionListChange;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

import javax.xml.bind.JAXBException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

public class MCAPLogSession extends Session
{
   // FIXME Figure out how to name the session
   private final String sessionName = getClass().getSimpleName();
   private final List<Robot> robots = new ArrayList<>();
   private final List<RobotDefinition> robotDefinitions = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();
   private MCAPFrameTransformBasedRobotStateUpdater robotStateUpdater = null;
   private final MCAPLogFileReader mcapLogFileReader;

   private final YoRegistry mcapRegistry = new YoRegistry("MCAP");

   /**
    * This is used to jump to a specific position in the log when the user drags the slider.
    * <p>
    * It is thread-safe.
    * </p>
    */
   private final AtomicInteger logPositionRequest = new AtomicInteger(-1);

   public MCAPLogSession(File mcapFile, long desiredLogDT, File robotModelFile) throws IOException
   {
      mcapLogFileReader = new MCAPLogFileReader(mcapFile, desiredLogDT, getInertialFrame(), mcapRegistry);
      mcapLogFileReader.loadSchemas();
      mcapLogFileReader.loadChannels();
      yoGraphicDefinitions.add(mcapLogFileReader.getYoGraphic());

      if (robotModelFile == null)
      {

         File[] modelFiles = mcapFile.getParentFile()
                                     .listFiles(file -> FilenameUtils.isExtension(file.getName(), "urdf") || FilenameUtils.isExtension(file.getName(), "sdf"));
         if (modelFiles != null && modelFiles.length == 1)
         {
            robotModelFile = modelFiles[0];
            LogTools.info("Found a robot model file in the same directory as the MCAP file: " + robotModelFile.getAbsolutePath());
         }
         else
         {
            LogTools.error(
                  "Could not find a robot model file in the same directory as the MCAP file: " + mcapFile.getAbsolutePath() + ", found candidates: " + (
                        modelFiles == null ?
                              "none" :
                              Arrays.toString(modelFiles)));
         }
      }

      RobotDefinition robotDefinition = null;
      if (robotModelFile != null)
         robotDefinition = loadRobotDefinition(robotModelFile);

      if (robotDefinition != null)
      {
         Robot robotToAdd = new Robot(robotDefinition, getInertialFrame());
         robots.add(robotToAdd);
         robotDefinitions.add(robotDefinition);
         rootRegistry.addChild(robotToAdd.getRegistry());
         robotStateUpdater = new MCAPFrameTransformBasedRobotStateUpdater(robotToAdd, mcapLogFileReader.getFrameTransformManager());
      }

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

         if (robotStateUpdater != null)
            robotStateUpdater.updateRobotState();
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

      if (robotStateUpdater != null)
         robotStateUpdater.updateRobotState();

      return mcapLogFileReader.getCurrentTimeInLog();
   }

   private boolean firstLogPositionRequest = true;

   @Override
   public void pauseTick()
   {
      if (firstPauseTick)
         firstLogPositionRequest = true;

      processRobotDefinitionRequests();

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
            robotStateUpdater.updateRobotState();

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

   private void processRobotDefinitionRequests()
   {
      SessionRobotDefinitionListChange request = pendingRobotDefinitionListChange.poll();

      if (request == null)
         return;

      RobotDefinition removedRobot = null;
      RobotDefinition addedRobot = null;

      if (request.getRemovedRobotDefinition() != null)
      {
         RobotDefinition robotDefinitionToRemove = request.getRemovedRobotDefinition();
         Robot robotToRemove = robots.stream().filter(robot -> robot.getRobotDefinition() == robotDefinitionToRemove).findFirst().orElse(null);
         if (robotToRemove != null)
         {
            robots.remove(robotToRemove);
            robotDefinitions.remove(robotDefinitionToRemove);
            robotStateUpdater = null;
            robotToRemove.destroy();
            removedRobot = robotDefinitionToRemove;
         }
      }

      RobotDefinition robotDefinitionToAdd = null;

      if (request.getNewRobotModelFile() != null)
         robotDefinitionToAdd = loadRobotDefinition(request.getNewRobotModelFile());
      else if (request.getAddedRobotDefinition() != null)
         robotDefinitionToAdd = request.getAddedRobotDefinition();

      if (robotDefinitionToAdd != null)
      {
         Robot robotToAdd = new Robot(robotDefinitionToAdd, getInertialFrame());
         robots.add(robotToAdd);
         robotDefinitions.add(robotDefinitionToAdd);
         rootRegistry.addChild(robotToAdd.getRegistry());
         robotStateUpdater = new MCAPFrameTransformBasedRobotStateUpdater(robotToAdd, mcapLogFileReader.getFrameTransformManager());

         // Update the robot state history
         YoBufferPropertiesReadOnly bufferProperties = getBufferProperties();
         int previousBufferIndex = bufferProperties.getCurrentIndex();
         int historyIndex = bufferProperties.getInPoint();

         for (int i = 0; i < bufferProperties.getActiveBufferLength(); i++)
         {
            sharedBuffer.setCurrentIndex(historyIndex);
            sharedBuffer.readBuffer();
            robotStateUpdater.updateRobotState();
            sharedBuffer.writeBuffer();
            historyIndex = SharedMemoryTools.increment(historyIndex, 1, bufferProperties.getSize());
         }

         // Go back to the previous buffer index
         sharedBuffer.setCurrentIndex(previousBufferIndex);
         sharedBuffer.readBuffer();
         robotStateUpdater.updateRobotState(); // Just to make sure the robot is updated.
         sharedBuffer.writeBuffer();

         addedRobot = robotDefinitionToAdd;
      }

      if (addedRobot != null && removedRobot != null)
      {
         reportRobotDefinitionListChange(SessionRobotDefinitionListChange.replace(addedRobot, removedRobot));
      }
      else if (addedRobot != null)
      {
         reportRobotDefinitionListChange(SessionRobotDefinitionListChange.add(addedRobot));
      }
      else if (removedRobot != null)
      {
         reportRobotDefinitionListChange(SessionRobotDefinitionListChange.remove(removedRobot));
      }
   }

   private static RobotDefinition loadRobotDefinition(File robotDefinitionFile)
   {
      if (FilenameUtils.isExtension(robotDefinitionFile.getName(), "urdf"))
      {
         try
         {
            URDFTools.URDFParserProperties urdfParserProperties = new URDFTools.URDFParserProperties();
            urdfParserProperties.setSimplifyKinematics(false);
            urdfParserProperties.setTransformToZUp(false);
            URDFModel urdfModel = URDFTools.loadURDFModel(robotDefinitionFile, Collections.singletonList(robotDefinitionFile.getParent()));
            RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, urdfParserProperties);

            robotDefinition.sanitizeNames();

            if (robotDefinition.getName() == null)
               robotDefinition.setName(FilenameUtils.getBaseName(robotDefinitionFile.getName()));

            return robotDefinition;
         }
         catch (JAXBException e)
         {
            LogTools.error("Failed to load URDF model from file: " + robotDefinitionFile.getAbsolutePath() + ".\n" + e.getMessage());
            return null;
         }
      }
      else if (FilenameUtils.isExtension(robotDefinitionFile.getName(), "sdf"))
      {
         try
         {
            SDFRoot sdfRoot = SDFTools.loadSDFRoot(robotDefinitionFile, Collections.singletonList(robotDefinitionFile.getParent()));
            return SDFTools.toFloatingRobotDefinition(sdfRoot.getModels().get(0));
         }
         catch (JAXBException e)
         {
            LogTools.error("Failed to load SDF model from file: " + robotDefinitionFile.getAbsolutePath() + ".\n" + e.getMessage());
            return null;
         }
      }
      else
      {
         LogTools.error("Unknown robot definition file extension: " + robotDefinitionFile.getName());
         return null;
      }
   }

   @Override
   public void submitRobotDefinitionListChange(SessionRobotDefinitionListChange change)
   {
      setSessionMode(SessionMode.PAUSE);
      super.submitRobotDefinitionListChange(change);
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
