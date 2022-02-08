package us.ihmc.scs2.simulation.physicsEngine;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface PhysicsEngine
{
   boolean initialize(Vector3DReadOnly gravity);

   void simulate(double currentTime, double dt, Vector3DReadOnly gravity);

   void pause();

   default Robot addRobot(RobotDefinition robotDefinition)
   {
      Robot robot = new Robot(robotDefinition, getInertialFrame());
      addRobot(robot);
      return robot;
   }

   void addRobot(Robot robot);

   void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition);

   ReferenceFrame getInertialFrame();

   List<? extends Robot> getRobots();

   List<RobotDefinition> getRobotDefinitions();

   List<TerrainObjectDefinition> getTerrainObjectDefinitions();

   default List<RobotStateDefinition> getCurrentRobotStateDefinitions()
   {
      return getRobots().stream().map(Robot::getCurrentRobotStateDefinition).collect(Collectors.toList());

   }

   List<RobotStateDefinition> getBeforePhysicsRobotStateDefinitions();

   YoRegistry getPhysicsEngineRegistry();
}
