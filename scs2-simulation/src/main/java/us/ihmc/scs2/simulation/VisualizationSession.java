package us.ihmc.scs2.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;

public class VisualizationSession extends Session
{
   public static final ReferenceFrame DEFAULT_INERTIAL_FRAME = ReferenceFrameTools.constructARootFrame("worldFrame");

   private final String sessionName;
   private final List<Robot> robots = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjects = new ArrayList<>();
   private final List<YoGraphicDefinition> yoGraphicDefinitions = new ArrayList<>();

   private final List<Controller> controllers = new ArrayList<>();

   public VisualizationSession()
   {
      this(retrieveCallerName());
   }

   public VisualizationSession(String sessionName)
   {
      this.sessionName = sessionName;
      setSessionModeTask(SessionMode.RUNNING, () ->
      {
         // Do Nothing, the user is responsible for invoking runTick();
      });
      setSessionMode(SessionMode.RUNNING);
      setSessionState(SessionState.ACTIVE);
   }

   @Override
   protected double doSpecificRunTick()
   {
      double dt = Conversions.nanosecondsToSeconds(getSessionDTNanoseconds());

      for (RobotInterface robot : robots)
      {
         RobotControllerManager controllerManager = robot.getControllerManager();
         controllerManager.updateControllers(time.getValue());
         controllerManager.writeControllerOutput(JointStateType.values());
      }

      for (Controller controller : controllers)
      {
         controller.doControl();
      }

      return time.getValue() + dt;
   }

   @Override
   protected void schedulingSessionMode(SessionMode previousMode, SessionMode newMode)
   {
      if (previousMode == newMode)
         return;

      if (previousMode == SessionMode.RUNNING)
      {
         for (RobotInterface robot : robots)
         {
            robot.getControllerManager().pauseControllers();
         }

         for (Controller controller : controllers)
         {
            controller.pause();
         }
      }
   }

   public void addController(Controller controller)
   {
      rootRegistry.addChild(controller.getYoRegistry());
      controllers.add(controller);
   }

   public void addRobot(Robot robot)
   {
      rootRegistry.addChild(robot.getRegistry());
      robots.add(robot);
   }

   public Robot addRobot(RobotDefinition robotDefinition)
   {
      Robot robot = new Robot(robotDefinition, DEFAULT_INERTIAL_FRAME);
      addRobot(robot);
      return robot;
   }

   @Override
   public String getSessionName()
   {
      return sessionName;
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      return robots.stream().map(Robot::getRobotDefinition).collect(Collectors.toList());
   }

   @Override
   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return terrainObjects;
   }

   @Override
   public List<YoGraphicDefinition> getYoGraphicDefinitions()
   {
      return yoGraphicDefinitions;
   }

   @Override
   public RobotStateDefinition getCurrentRobotStateDefinition(RobotDefinition robotDefinition)
   {
      Robot robot = robots.stream().filter(candidate -> candidate.getRobotDefinition() == robotDefinition).findFirst().orElse(null);
      if (robot == null)
         return null;
      return extractRobotState(robot.getName(), robot.getRootBody());
   }
}
