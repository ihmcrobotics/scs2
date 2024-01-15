package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.RobotJointWrenchCalculator;
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

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class ContactPointBasedPhysicsEngine implements PhysicsEngine
{
   /**
    * The maximum translational acceleration this joint may undergo before throwing an
    * {@link IllegalStateException}.
    */
   public static final double MAX_TRANS_ACCEL = 1000000000000.0;

   /**
    * The maximum rotational acceleration this joint may undergo before throwing an
    * {@link IllegalStateException}.
    */
   public static final double MAX_ROT_ACCEL = 10000000.0;

   private final ReferenceFrame inertialFrame;

   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());

   private final List<ContactPointBasedRobot> robotList = new ArrayList<>();
   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private final List<Collidable> environmentCollidables = new ArrayList<>();

   private final ContactPointBasedForceCalculator forceCalculator;

   private boolean estimateJointWrenches = false;
   private boolean hasBeenInitialized = false;

   public ContactPointBasedPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry)
   {
      this.inertialFrame = inertialFrame;
      this.rootRegistry = rootRegistry;

      forceCalculator = new ContactPointBasedForceCalculator(inertialFrame, physicsEngineRegistry);
   }

   /**
    * Whether to estimate the joint wrenches or not.
    * <p>
    * Estimating the joint wrenches is useful for estimating forces going through the robot limbs and can be used
    * to do FEA analysis.
    * </p>
    * <p>
    * When enabled, the joint wrenches are displayed in the simulation GUI under the {@link RobotJointWrenchCalculator} registry.
    * </p>
    *
    * @param estimateJointWrenches {@code true} for estimating the joint wrenches, {@code false} otherwise.
    */
   public void setEstimateJointWrenches(boolean estimateJointWrenches)
   {
      this.estimateJointWrenches = estimateJointWrenches;
   }

   @Override
   public void initialize(Vector3DReadOnly gravity)
   {
      for (ContactPointBasedRobot robot : robotList)
      {
         robot.initializeState();
         robot.resetCalculators();
         // Fill out the joint accelerations so the accelerometers can get initialized.
         robot.doForwardDynamics(gravity);
         robot.updateSensors();
         robot.getControllerManager().initializeControllers();
      }
      forceCalculator.reset(robotList);
      hasBeenInitialized = true;
   }

   private final Wrench tempWrench = new Wrench();

   @Override
   public void simulate(double currentTime, double dt, Vector3DReadOnly gravity)
   {
      if (!hasBeenInitialized)
      {
         initialize(gravity);
         return;
      }

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
            List<GroundContactPoint> groundContactPoints = joint.getAuxiliaryData().getGroundContactPoints();
            List<ExternalWrenchPoint> externalWrenchPoints = joint.getAuxiliaryData().getExternalWrenchPoints();

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
         robot.computeJointWrenches(dt);

         robot.getAllJoints().forEach(joint ->
                                      {
                                         if (joint.getJointTwist().getAngularPart().norm() > MAX_ROT_ACCEL)
                                            throw new IllegalStateException("Unreasonable acceleration for the joint " + joint);
                                         if (joint.getJointTwist().getLinearPart().norm() > MAX_TRANS_ACCEL)
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
      if (estimateJointWrenches)
         cpbRobot.enableJointWrenchCalculator();
      rootRegistry.addChild(cpbRobot.getRegistry());
      physicsEngineRegistry.addChild(cpbRobot.getSecondaryRegistry());
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
