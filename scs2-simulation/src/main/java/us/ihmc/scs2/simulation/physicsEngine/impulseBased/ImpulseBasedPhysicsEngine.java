package us.ihmc.scs2.simulation.physicsEngine.impulseBased;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.RobotStateDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.YoTimer;
import us.ihmc.scs2.simulation.RobotJointWrenchCalculator;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollisionTools;
import us.ihmc.scs2.simulation.parameters.ConstraintParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.YoConstraintParameters;
import us.ihmc.scs2.simulation.parameters.YoContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.MultiRobotCollisionGroup;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.SimpleCollisionDetection;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.scs2.simulation.shapes.FrameSTPBox3D;
import us.ihmc.scs2.simulation.shapes.FrameSTPCylinder3D;
import us.ihmc.scs2.simulation.shapes.interfaces.FrameSTPBox3DReadOnly;
import us.ihmc.scs2.simulation.shapes.interfaces.FrameSTPCylinder3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

/**
 * Physics engine that simulates the dynamic behavior of multiple robots and their contact
 * interactions.
 * <p>
 * Its uses Featherstone forward dynamics algorithm to account for controller outputs, i.e. joint
 * efforts. The interactions are simulated using an impulse-based framework.
 * </p>
 * <p>
 * References that this physics engine is based on:
 * <ul>
 * <li>Multi-body forward dynamics algorithm: Featherstone, Roy. <i>Rigid Body Dynamics
 * Algorithms</i> Springer, 2008.
 * <li>Multi-body impulse response algorithm: Mirtich, Brian Vincent. <i>Impulse-based dynamic
 * simulation of rigid body systems</i>. University of California, Berkeley, 1996.
 * <li>Impulse-based contact resolution algorithm: Hwangbo, Jemin, Joonho Lee, and Marco Hutter.
 * <i>Per-contact iteration method for solving contact dynamics.</i> IEEE Robotics and Automation
 * Letters 3.2 (2018): 895-902.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class ImpulseBasedPhysicsEngine implements PhysicsEngine
{
   private final ReferenceFrame inertialFrame;

   private final YoRegistry rootRegistry;
   private final YoRegistry physicsEngineRegistry = new YoRegistry(getClass().getSimpleName());
   private final List<ImpulseBasedRobot> robotList = new ArrayList<>();
   private final Map<RigidBodyBasics, ImpulseBasedRobot> robotMap = new HashMap<>();
   private final YoMultiContactImpulseCalculatorPool multiContactImpulseCalculatorPool;

   private List<MultiRobotCollisionGroup> collisionGroups;

   private final List<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private final List<Collidable> environmentCollidables = new ArrayList<>();
   /** Margins for rounding the robots' sharp collision shapes, see {@link #roundSharpCollisionShapes(Robot)}. */
   private double shapeRoundingMinimumMargin = 1.0e-5;
   private double shapeRoundingMaximumMargin = 4.0e-4;

   private final SimpleCollisionDetection collisionDetectionPlugin;

   private final YoBoolean hasGlobalContactParameters;
   private final YoContactParameters globalContactParameters;
   private final YoBoolean hasGlobalConstraintParameters;
   private final YoConstraintParameters globalConstraintParameters;

   private final YoRegistry physicsEngineStatisticsRegistry = new YoRegistry("physicsEngineStatistics");
   private final YoTimer physicsEngineTotalTimer = new YoTimer("physicsEngineTotalTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoDouble physicsEngineRealTimeRate = new YoDouble("physicsEngineRealTimeRate", physicsEngineStatisticsRegistry);

   private boolean estimateJointWrenches = false;
   private boolean hasBeenInitialized = false;

   private MultiContactImpulseCalculatorStepListener multiContactCalculatorStepListener;

   public ImpulseBasedPhysicsEngine(ReferenceFrame inertialFrame, YoRegistry rootRegistry)
   {
      this.rootRegistry = rootRegistry;
      this.inertialFrame = inertialFrame;

      physicsEngineRegistry.addChild(physicsEngineStatisticsRegistry);

      collisionDetectionPlugin = new SimpleCollisionDetection(inertialFrame);

      YoRegistry multiContactCalculatorRegistry = new YoRegistry(MultiContactImpulseCalculator.class.getSimpleName());
      physicsEngineRegistry.addChild(multiContactCalculatorRegistry);

      hasGlobalContactParameters = new YoBoolean("hasGlobalContactParameters", physicsEngineRegistry);
      globalContactParameters = new YoContactParameters("globalContact", physicsEngineRegistry);
      hasGlobalConstraintParameters = new YoBoolean("hasGlobalConstraintParameters", physicsEngineRegistry);
      globalConstraintParameters = new YoConstraintParameters("globalConstraint", physicsEngineRegistry);
      multiContactImpulseCalculatorPool = new YoMultiContactImpulseCalculatorPool(1, inertialFrame, multiContactCalculatorRegistry);
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

   public void setMultiContactCalculatorStepListener(MultiContactImpulseCalculatorStepListener multiContactCalculatorStepListener)
   {
      this.multiContactCalculatorStepListener = multiContactCalculatorStepListener;
   }

   @Override
   public void addTerrainObject(TerrainObjectDefinition terrainObjectDefinition)
   {
      terrainObjectDefinitions.add(terrainObjectDefinition);
      environmentCollidables.addAll(CollisionTools.toCollisionShape(terrainObjectDefinition, inertialFrame));
   }

   @Override
   public void addRobot(Robot robot)
   {
      inertialFrame.checkReferenceFrameMatch(robot.getInertialFrame());
      roundSharpCollisionShapes(robot);
      ImpulseBasedRobot ibRobot = new ImpulseBasedRobot(robot, physicsEngineRegistry);
      if (estimateJointWrenches)
         ibRobot.enableJointWrenchCalculator();
      robotMap.put(ibRobot.getRootBody(), ibRobot);
      rootRegistry.addChild(ibRobot.getRegistry());
      physicsEngineRegistry.addChild(ibRobot.getSecondaryRegistry());
      robotList.add(ibRobot);
   }

   /**
    * Sets the margins used to round the robots' sharp collision shapes, see {@link #roundSharpCollisionShapes(Robot)}.
    * Setting both to zero disables the rounding, which is only advisable if the shapes are already smooth.
    *
    * @param minimumMargin the smallest distance between the original shape and the rounded one.
    * @param maximumMargin the largest distance between the original shape and the rounded one.
    */
   public void setCollisionShapeRoundingMargins(double minimumMargin, double maximumMargin)
   {
      shapeRoundingMinimumMargin = minimumMargin;
      shapeRoundingMaximumMargin = maximumMargin;
   }

   /**
    * Replaces the robot's boxes and cylinders with their rounded (sphere-torus-patch) equivalents.
    * <p>
    * This engine resolves a single contact point per pair of shapes. Two flat faces resting on each other do not
    * define one: the contact point jumps between the face's features from tick to tick, and the shapes sink into each
    * other rather than resting. Rounding the edges by a fraction of a millimetre gives the closest-point query a
    * unique and continuous solution, at a cost in size that is well below the accuracy of the collision geometry
    * itself.
    * </p>
    * <p>
    * Spheres, capsules and ellipsoids are already smooth and are left alone, as are shapes that are already rounded.
    * Only the robots are treated: rounding one side of each contact pair is enough, and the terrain is commonly a
    * height map rather than shapes.
    * </p>
    */
   private void roundSharpCollisionShapes(Robot robot)
   {
      if (shapeRoundingMinimumMargin <= 0.0 && shapeRoundingMaximumMargin <= 0.0)
         return;

      for (SimRigidBodyBasics rigidBody : robot.getRootBody().subtreeIterable())
      {
         List<Collidable> collidables = rigidBody.getCollidables();

         for (int i = 0; i < collidables.size(); i++)
         {
            Collidable collidable = collidables.get(i);
            FrameShape3DReadOnly roundedShape = roundSharpEdges(collidable.getShape());

            if (roundedShape != collidable.getShape())
               collidables.set(i, new Collidable(collidable.getRigidBody(), collidable.getCollisionMask(), collidable.getCollisionGroup(), roundedShape));
         }
      }
   }

   private FrameShape3DReadOnly roundSharpEdges(FrameShape3DReadOnly shape)
   {
      if (shape instanceof FrameSTPBox3DReadOnly || shape instanceof FrameSTPCylinder3DReadOnly)
         return shape;

      if (shape instanceof FrameBox3DReadOnly box)
      {
         FrameSTPBox3D roundedBox = new FrameSTPBox3D(box);
         roundedBox.setMargins(shapeRoundingMinimumMargin, shapeRoundingMaximumMargin);
         return roundedBox;
      }

      if (shape instanceof FrameCylinder3DReadOnly cylinder)
      {
         FrameSTPCylinder3D roundedCylinder = new FrameSTPCylinder3D(cylinder);
         roundedCylinder.setMargins(shapeRoundingMinimumMargin, shapeRoundingMaximumMargin);
         return roundedCylinder;
      }

      return shape;
   }

   public void setGlobalConstraintParameters(ConstraintParametersReadOnly parameters)
   {
      globalConstraintParameters.set(parameters);
      hasGlobalConstraintParameters.set(true);
   }

   public void setGlobalContactParameters(ContactParametersReadOnly parameters)
   {
      globalContactParameters.set(parameters);
      hasGlobalContactParameters.set(true);
   }

   @Override
   public void initialize(Vector3DReadOnly gravity)
   {
      for (ImpulseBasedRobot robot : robotList)
      {
         robot.resetDT();
         robot.resetState();
         robot.initializeState();
         robot.resetCalculators();
         // Fill out the joint accelerations so the accelerometers can get initialized.
         robot.doForwardDynamics(gravity);
         robot.updateSensors();
         robot.getControllerManager().initializeControllers();
      }
      collisionDetectionPlugin.clear();

      hasBeenInitialized = true;
   }

   private final YoTimer initialPhaseTimer = new YoTimer("initialPhaseTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoTimer detectCollisionsTimer = new YoTimer("detectCollisionsTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoTimer configureCollisionHandlersTimer = new YoTimer("configureCollisionHandlersTimer",
                                                                       TimeUnit.MILLISECONDS,
                                                                       physicsEngineStatisticsRegistry);
   private final YoTimer handleCollisionsTimer = new YoTimer("handleCollisionsTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);
   private final YoTimer finalPhaseTimer = new YoTimer("finalPhaseTimer", TimeUnit.MILLISECONDS, physicsEngineStatisticsRegistry);

   private final Wrench tempWrench = new Wrench();

   @Override
   public void simulate(double currentTime, double dt, Vector3DReadOnly gravity)
   {
      if (!hasBeenInitialized)
      {
         initialize(gravity);
         return;
      }

      physicsEngineTotalTimer.start();
      initialPhaseTimer.start();

      for (ImpulseBasedRobot robot : robotList)
      {
         robot.updateFrames();
         robot.updateSensors();
         robot.resetCalculators();
         robot.getControllerManager().updateControllers(currentTime);
         robot.getControllerManager().writeControllerOutput(JointStateType.EFFORT);
         robot.getControllerManager().writeControllerOutputForJointsToIgnore(JointStateType.values());
         robot.saveRobotBeforePhysicsState();
      }

      for (ImpulseBasedRobot robot : robotList)
      {
         robot.updateCollidableBoundingBoxes();
         robot.computeJointLowLevelControl();

         for (SimJointBasics joint : robot.getJointsToConsider())
         {
            List<ExternalWrenchPoint> externalWrenchPoints = joint.getAuxiliaryData().getExternalWrenchPoints();

            if (externalWrenchPoints.isEmpty())
               continue;

            SimRigidBodyBasics body = joint.getSuccessor();
            FixedFrameWrenchBasics externalWrench = robot.getForwardDynamicsCalculator().getExternalWrench(body);

            for (ExternalWrenchPoint efp : externalWrenchPoints)
            {
               tempWrench.setIncludingFrame(efp.getWrench());
               tempWrench.changeFrame(externalWrench.getReferenceFrame());
               externalWrench.add(tempWrench);
            }
         }

         robot.doForwardDynamics(gravity);
      }

      environmentCollidables.forEach(collidable -> collidable.updateBoundingBox(inertialFrame));
      initialPhaseTimer.stop();
      detectCollisionsTimer.start();

      if (hasGlobalContactParameters.getValue())
         collisionDetectionPlugin.setMinimumPenetration(globalContactParameters.getMinimumPenetration());
      collisionDetectionPlugin.evaluationCollisions(robotList, () -> environmentCollidables, dt);

      collisionGroups = MultiRobotCollisionGroup.toCollisionGroups(collisionDetectionPlugin.getAllCollisions());

      detectCollisionsTimer.stop();
      configureCollisionHandlersTimer.start();

      Set<RigidBodyBasics> uncoveredRobotsRootBody = new HashSet<>(robotMap.keySet());
      List<MultiContactImpulseCalculator> impulseCalculators = new ArrayList<>();

      multiContactImpulseCalculatorPool.clear();

      for (MultiRobotCollisionGroup collisionGroup : collisionGroups)
      {
         MultiContactImpulseCalculator calculator = multiContactImpulseCalculatorPool.nextAvailable();

         calculator.configure(robotMap, collisionGroup);

         if (hasGlobalConstraintParameters.getValue())
            calculator.setConstraintParameters(globalConstraintParameters);
         if (hasGlobalContactParameters.getValue())
            calculator.setContactParameters(globalContactParameters);
         if (multiContactCalculatorStepListener != null)
            calculator.setListener(multiContactCalculatorStepListener);

         impulseCalculators.add(calculator);
         uncoveredRobotsRootBody.removeAll(collisionGroup.getRootBodies());
      }

      configureCollisionHandlersTimer.stop();
      handleCollisionsTimer.start();

      for (RigidBodyBasics rootBody : uncoveredRobotsRootBody)
      {
         ImpulseBasedRobot robot = robotMap.get(rootBody);
         RobotJointLimitImpulseBasedCalculator jointLimitConstraintCalculator = robot.getJointLimitConstraintCalculator();
         jointLimitConstraintCalculator.initialize(dt);
         jointLimitConstraintCalculator.updateInertia(null, null);
         jointLimitConstraintCalculator.computeImpulse(dt);
         robot.addJointVelocityChange(jointLimitConstraintCalculator.getJointVelocityChange(0));
      }

      for (MultiContactImpulseCalculator impulseCalculator : impulseCalculators)
      {
         impulseCalculator.computeImpulses(currentTime, dt, false);
         impulseCalculator.writeJointDeltaVelocities();
         impulseCalculator.writeImpulses();
         impulseCalculator.setListener(null);
      }

      handleCollisionsTimer.stop();
      finalPhaseTimer.start();

      for (ImpulseBasedRobot robot : robotList)
      {
         robot.writeJointAccelerations();
         robot.writeJointDeltaVelocities();
         robot.computeJointWrenches(dt);
         robot.integrateState(dt);
      }

      finalPhaseTimer.stop();
      physicsEngineTotalTimer.stop();
      physicsEngineRealTimeRate.set(dt * 1.0e3 / physicsEngineTotalTimer.getTimer().getValue());
   }

   @Override
   public void pause()
   {
      for (ImpulseBasedRobot robot : robotList)
      {
         robot.updateFrames();
         robot.updateSensors();
         robot.getControllerManager().pauseControllers();
      }
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   @Override
   public List<Robot> getRobots()
   {
      return robotList.stream().map(ImpulseBasedRobot::getRobot).collect(Collectors.toList());
   }

   @Override
   public List<RobotDefinition> getRobotDefinitions()
   {
      return robotList.stream().map(ImpulseBasedRobot::getRobotDefinition).collect(Collectors.toList());
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
