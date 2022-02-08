package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollisionTools;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParametersReadOnly;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ContactPointBasedPhysicsEngine implements PhysicsEngine
{
   /**
    * The maximum translational acceleration this joint may undergo before throwing an
    * {@link UnreasonableAccelerationException UnreasonableAccelerationException}.
    */
   public static final double MAX_TRANS_ACCEL = 1000000000000.0;

   /**
    * The maximum rotational acceleration this joint may undergo before throwing an
    * {@link UnreasonableAccelerationException UnreasonableAccelerationException}.
    */
   public static final double MAX_ROT_ACCEL = 10000000.0;

   private final ReferenceFrame inertialFrame;

   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());

   private final List<ContactPointBasedRobot> robotList = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private final List<Collidable> environmentCollidables = new ArrayList<>();

   private final ContactPointBasedForceCalculator forceCalculator;

   private boolean initialize = true;

   public ContactPointBasedPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry)
   {
      this.inertialFrame = inertialFrame;
      this.rootRegistry = rootRegistry;

      forceCalculator = new ContactPointBasedForceCalculator(inertialFrame, physicsEngineRegistry);
   }

   @Override
   public boolean initialize(Vector3DReadOnly gravity)
   {
      if (!initialize)
         return false;

      for (ContactPointBasedRobot robot : robotList)
      {
         robot.initializeState();
         robot.resetCalculators();
         // Fill out the joint accelerations so the accelerometers can get initialized.
         robot.doForwardDynamics(gravity);
         robot.updateSensors();
         robot.getControllerManager().initializeControllers();
      }
      initialize = false;
      return true;
   }

   private final Wrench tempWrench = new Wrench();

   @Override
   public void simulate(double currentTime, double dt, Vector3DReadOnly gravity)
   {
      if (initialize(gravity))
         return;

      for (ContactPointBasedRobot robot : robotList)
      {
         robot.resetCalculators();
         robot.getControllerManager().updateControllers(currentTime);
         robot.getControllerManager().writeControllerOutput(JointStateType.EFFORT);
         robot.getControllerManager().writeControllerOutputForJointsToIgnore(JointStateType.values());
         robot.saveRobotBeforePhysicsState();
      }

      for (ContactPointBasedRobot robot : robotList)
      {
         robot.computeJointDamping();
         robot.computeJointSoftLimits();
         robot.updateCollidableBoundingBoxes();
      }

      environmentCollidables.forEach(collidable -> collidable.updateBoundingBox(inertialFrame));
      forceCalculator.resolveContactForces(robotList, () -> environmentCollidables);

      for (ContactPointBasedRobot robot : robotList)
      {
         for (SimJointBasics joint : robot.getJointsToConsider())
         {
            List<GroundContactPoint> groundContactPoints = joint.getAuxialiryData().getGroundContactPoints();
            List<ExternalWrenchPoint> externalWrenchPoints = joint.getAuxialiryData().getExternalWrenchPoints();

            if (groundContactPoints.isEmpty() && externalWrenchPoints.isEmpty())
               continue;

            SimRigidBodyBasics body = joint.getSuccessor();
            FixedFrameWrenchBasics externalWrench = robot.getForwardDynamicsCalculator().getExternalWrench(body);

            for (GroundContactPoint gcp : groundContactPoints)
            {
               tempWrench.setIncludingFrame(gcp.getWrench());
               tempWrench.changeFrame(externalWrench.getReferenceFrame());
               externalWrench.add(tempWrench);
            }

            for (ExternalWrenchPoint efp : externalWrenchPoints)
            {
               tempWrench.setIncludingFrame(efp.getWrench());
               tempWrench.changeFrame(externalWrench.getReferenceFrame());
               externalWrench.add(tempWrench);
            }

            robot.addRigidBodyExternalWrench(body, externalWrench);
         }

         robot.doForwardDynamics(gravity);
      }

      for (ContactPointBasedRobot robot : robotList)
      {
         robot.writeJointAccelerations();

         robot.getAllJoints().forEach(joint ->
         {
            if (joint.getJointTwist().getAngularPart().length() > MAX_ROT_ACCEL)
               throw new IllegalStateException("Unreasonable acceleration for the joint " + joint);
            if (joint.getJointTwist().getLinearPart().length() > MAX_TRANS_ACCEL)
               throw new IllegalStateException("Unreasonable acceleration for the joint " + joint);
         });

         robot.integrateState(dt);
         robot.updateFrames();
         robot.updateSensors();
      }
   }

   @Override
   public void pause()
   {
      for (ContactPointBasedRobot robot : robotList)
      {
         robot.getControllerManager().pauseControllers();
      }
   }

   public void setGroundContactParameters(ContactPointBasedContactParametersReadOnly parameters)
   {
      forceCalculator.setParameters(parameters);
   }

   @Override
   public void addRobot(Robot robot)
   {
      inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());
      ContactPointBasedRobot cpbRobot = new ContactPointBasedRobot(robot, physicsEngineRegistry);
      rootRegistry.addChild(cpbRobot.getRegistry());
      robotList.add(cpbRobot);
   }

   @Override
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      terrainObjectDefinitions.add(terrainObjectDefinition);
      environmentCollidables.addAll(CollisionTools.toCollisionShape(terrainObjectDefinition, inertialFrame));
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   @Override
   public List<Robot> getRobots()
   {
      return robotList.stream().map(ContactPointBasedRobot::getRobot).collect(Collectors.toList());
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      return robotList.stream().map(RobotInterface::getRobotDefinition).collect(Collectors.toList());
   }

   @Override
   public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
   {
      return terrainObjectDefinitions;
   }

   @Override
   public List<RobotStateDefinition> getBeforePhysicsRobotStateDefinitions()
   {
      return robotList.stream().map(RobotExtension::getRobotBeforePhysicsStateDefinition).collect(Collectors.toList());
   }

   @Override
   public YoRegistry getPhysicsEngineRegistry()
   {
      return physicsEngineRegistry;
   }
}
