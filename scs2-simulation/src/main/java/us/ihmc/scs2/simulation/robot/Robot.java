package us.ihmc.scs2.simulation.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.PlanarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SphericalJointReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.robot.RobotStateDefinition.JointStateEntry;
import us.ihmc.scs2.definition.state.JointState;
import us.ihmc.scs2.definition.state.JointStateBase;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.PlanarJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.SphericalJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.controller.LoopClosureSoftConstraintController;
import us.ihmc.scs2.simulation.robot.controller.RobotControllerManager;
import us.ihmc.scs2.simulation.robot.multiBodySystem.*;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class Robot implements RobotInterface
{
   public static final JointBuilderFromDefinition DEFAULT_JOINT_BUILDER = new JointBuilderFromDefinition()
   {
   };
   public static final RigidBodyBuilderFromDefinition DEFAULT_BODY_BUILDER = new RigidBodyBuilderFromDefinition()
   {
   };

   protected final YoRegistry registry;

   protected final RobotDefinition robotDefinition;
   protected final String name;
   protected final SimRigidBody rootBody;
   protected final ReferenceFrame inertialFrame;
   protected final ReferenceFrame robotRootFrame;

   protected final List<SimJointBasics> allJoints;
   protected final List<SimRigidBodyBasics> allRigidBodies;
   protected final List<SimJointBasics> jointsToIgnore;
   protected final List<SimJointBasics> jointsToConsider;
   protected final Map<String, SimJointBasics> nameToJointMap;
   protected final Map<String, SimRigidBodyBasics> nameToBodyMap;
   protected final JointMatrixIndexProvider jointMatrixIndexProvider;
   protected final List<JointStateReadOnly> allJointInitialStates;

   protected final RobotControllerManager controllerManager;

   /**
    * Creates a new robot than can be used for instance for a simulation.
    * <p>
    * For simulation, make sure to use the same inertial frame as the session. By default, a simulation
    * session will use {@link SimulationSession#DEFAULT_INERTIAL_FRAME}.
    * </p>
    * 
    * @param robotDefinition the template of the robot.
    * @param inertialFrame   the global to use for this robot, typically is
    *                        {@link SimulationSession#DEFAULT_INERTIAL_FRAME}.
    */
   public Robot(RobotDefinition robotDefinition, ReferenceFrame inertialFrame)
   {
      this(robotDefinition, inertialFrame, true);
   }

   /**
    * Creates a new robot than can be used for instance for a simulation.
    * <p>
    * For simulation, make sure to use the same inertial frame as the session. By default, a simulation
    * session will use {@link SimulationSession#DEFAULT_INERTIAL_FRAME}.
    * </p>
    * 
    * @param robotDefinition the template of the robot.
    * @param inertialFrame   the global to use for this robot, typically is
    *                        {@link SimulationSession#DEFAULT_INERTIAL_FRAME}.
    * @param loadControllers whether the instantiate the {@link ControllerDefinition}s associated to
    *                        the given {@code robotDefinion}. Typically equal to {@code true}.
    */
   public Robot(RobotDefinition robotDefinition, ReferenceFrame inertialFrame, boolean loadControllers)
   {
      this.robotDefinition = robotDefinition;
      this.inertialFrame = inertialFrame;
      robotRootFrame = createRobotRootFrame(robotDefinition, inertialFrame);

      name = robotDefinition.getName();

      registry = new YoRegistry(name);

      rootBody = createRobot(robotDefinition.getRootBodyDefinition(), robotRootFrame, DEFAULT_JOINT_BUILDER, DEFAULT_BODY_BUILDER, registry);
      allJoints = SubtreeStreams.fromChildren(SimJointBasics.class, rootBody).collect(Collectors.toList());
      allRigidBodies = new ArrayList<>(rootBody.subtreeList());
      nameToJointMap = allJoints.stream().collect(Collectors.toMap(SimJointBasics::getName, Function.identity()));
      nameToBodyMap = allRigidBodies.stream().collect(Collectors.toMap(SimRigidBodyBasics::getName, Function.identity()));
      jointsToIgnore = robotDefinition.getNameOfJointsToIgnore().stream().map(nameToJointMap::get).collect(Collectors.toList());
      jointsToConsider = allJoints.stream().filter(joint -> !jointsToIgnore.contains(joint)).collect(Collectors.toList());
      jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(getJointsToConsider());
      allJointInitialStates = allJoints.stream()
                                       .map(joint -> robotDefinition.getJointDefinition(joint.getName()).getInitialJointState())
                                       .collect(Collectors.toList());

      controllerManager = new RobotControllerManager(this, registry);

      if (loadControllers)
      {
         createSoftConstraintControllerDefinitions(robotDefinition).forEach(controllerManager::addController);
         robotDefinition.getControllerDefinitions().forEach(controllerManager::addController);
      }
   }

   public static ReferenceFrame createRobotRootFrame(RobotDefinition robotDefinition, ReferenceFrame inertialFrame)
   {
      return MovingReferenceFrame.constructFrameFixedInParent(robotDefinition.getName() + "RootFrame", inertialFrame, new RigidBodyTransform());
   }

   public static SimRigidBody createRobot(RigidBodyDefinition rootBodyDefinition,
                                          ReferenceFrame inertialFrame,
                                          JointBuilderFromDefinition jointBuilder,
                                          RigidBodyBuilderFromDefinition bodyBuilder,
                                          YoRegistry registry)
   {
      SimRigidBody rootBody = bodyBuilder.rootFromDefinition(rootBodyDefinition, inertialFrame, registry);
      createJointsRecursive(rootBody, rootBodyDefinition, jointBuilder, bodyBuilder, registry);
      RobotDefinition.closeLoops(rootBody, rootBodyDefinition);
      return rootBody;
   }

   public static void createJointsRecursive(SimRigidBody rigidBody,
                                            RigidBodyDefinition rigidBodyDefinition,
                                            JointBuilderFromDefinition jointBuilder,
                                            RigidBodyBuilderFromDefinition bodyBuilder,
                                            YoRegistry registry)
   {
      for (JointDefinition childJointDefinition : rigidBodyDefinition.getChildrenJoints())
      {
         SimJointBasics childJoint = jointBuilder.fromDefinition(childJointDefinition, rigidBody);

         if (childJointDefinition.isLoopClosure())
            continue;

         SimRigidBody childSuccessor = bodyBuilder.fromDefinition(childJointDefinition.getSuccessor(), childJoint);

         childJointDefinition.getKinematicPointDefinitions().forEach(childJoint.getAuxiliaryData()::addKinematicPoint);
         childJointDefinition.getExternalWrenchPointDefinitions().forEach(childJoint.getAuxiliaryData()::addExternalWrenchPoint);
         childJointDefinition.getGroundContactPointDefinitions().forEach(childJoint.getAuxiliaryData()::addGroundContactPoint);
         for (SensorDefinition sensorDefinition : childJointDefinition.getSensorDefinitions())
         {
            if (sensorDefinition instanceof IMUSensorDefinition)
               childJoint.getAuxiliaryData().addIMUSensor((IMUSensorDefinition) sensorDefinition);
            else if (sensorDefinition instanceof WrenchSensorDefinition)
               childJoint.getAuxiliaryData().addWrenchSensor((WrenchSensorDefinition) sensorDefinition);
            else if (sensorDefinition instanceof CameraSensorDefinition)
               childJoint.getAuxiliaryData().addCameraSensor((CameraSensorDefinition) sensorDefinition);
            else
               LogTools.warn("Unsupported sensor: " + sensorDefinition);
         }

         createJointsRecursive(childSuccessor, childJointDefinition.getSuccessor(), jointBuilder, bodyBuilder, registry);
      }
   }

   public static List<ControllerDefinition> createSoftConstraintControllerDefinitions(RobotDefinition robotDefinition)
   {
      List<ControllerDefinition> controllerDefinitions = new ArrayList<>();

      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         if (!jointDefinition.isLoopClosure())
            continue;

         controllerDefinitions.add((controllerInput, controllerOutput) ->
         {
            String name = jointDefinition.getName();
            LoopClosureDefinition loopClosureDefinition = jointDefinition.getLoopClosureDefinition();
            RigidBodyTransformReadOnly transformToParentJoint = jointDefinition.getTransformToParent();
            RigidBodyTransformReadOnly transformToSuccessorParentJoint = loopClosureDefinition.getTransformToSuccessorParent();

            Matrix3DReadOnly constraintForceSubSpace = LoopClosureDefinition.jointForceSubSpace(jointDefinition);
            Matrix3DReadOnly constraintMomentSubSpace = LoopClosureDefinition.jointMomentSubSpace(jointDefinition);
            if (constraintForceSubSpace == null || constraintMomentSubSpace == null)
               throw new UnsupportedOperationException("Loop closure not supported for " + jointDefinition);

            LoopClosureSoftConstraintController constraint = new LoopClosureSoftConstraintController(name,
                                                                                                     transformToParentJoint,
                                                                                                     transformToSuccessorParentJoint,
                                                                                                     constraintForceSubSpace,
                                                                                                     constraintMomentSubSpace);
            constraint.setParentJoint((SimJointBasics) controllerInput.getInput().findJoint(jointDefinition.getParentJoint().getName()));
            constraint.setSuccessor((SimRigidBodyBasics) controllerInput.getInput().findRigidBody(jointDefinition.getSuccessor().getName()));
            constraint.setGains(loopClosureDefinition.getKpSoftConstraint(), loopClosureDefinition.getKdSoftConstraint());
            return constraint;
         });
      }

      return controllerDefinitions;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   @Override
   public RobotControllerManager getControllerManager()
   {
      return controllerManager;
   }

   @Override
   public void resetState()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         allJoints.get(i).resetState();
      }
      rootBody.updateFramesRecursively();
   }

   @Override
   public void initializeState()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         SimJointBasics jointToUpdate = allJoints.get(i);
         JointStateReadOnly initialState = allJointInitialStates.get(i);
         if (initialState != null)
            initialState.getAllStates(jointToUpdate);
      }
      rootBody.updateFramesRecursively();
   }

   @Override
   public SimRigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   @Override
   public SimRigidBodyBasics getRigidBody(String name)
   {
      return nameToBodyMap.get(name);
   }

   @Override
   public SimJointBasics getJoint(String name)
   {
      return nameToJointMap.get(name);
   }

   @Override
   public List<? extends SimJointBasics> getAllJoints()
   {
      return allJoints;
   }

   @Override
   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }

   @Override
   public JointMatrixIndexProvider getJointMatrixIndexProvider()
   {
      return jointMatrixIndexProvider;
   }

   @Override
   public List<? extends SimJointBasics> getJointsToConsider()
   {
      return jointsToConsider;
   }

   @Override
   public List<? extends SimJointBasics> getJointsToIgnore()
   {
      return jointsToIgnore;
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   public RobotStateDefinition getCurrentRobotStateDefinition()
   {
      RobotStateDefinition definition = new RobotStateDefinition();
      definition.setRobotName(name);
      List<JointStateEntry> jointStateEntries = new ArrayList<>();

      for (JointReadOnly joint : rootBody.childrenSubtreeIterable())
      {
         jointStateEntries.add(new JointStateEntry(joint.getName(), extractJointState(joint)));
      }

      definition.setJointStateEntries(jointStateEntries);
      return definition;
   }

   private static JointStateBase extractJointState(JointReadOnly joint)
   {
      if (joint == null)
         return null;

      JointStateBase state;
      if (joint instanceof OneDoFJointReadOnly)
         state = new OneDoFJointState();
      else if (joint instanceof SixDoFJointReadOnly)
         state = new SixDoFJointState();
      else if (joint instanceof SphericalJointReadOnly)
         state = new SphericalJointState();
      else if (joint instanceof PlanarJointReadOnly)
         state = new PlanarJointState();
      else
         state = new JointState(joint.getConfigurationMatrixSize(), joint.getDegreesOfFreedom());

      state.setConfiguration(joint);
      state.setVelocity(joint);
      state.setAcceleration(joint);
      state.setEffort(joint);
      return state;
   }

   public static interface JointBuilderFromDefinition
   {
      default SimJointBasics fromDefinition(JointDefinition definition, SimRigidBody predecessor)
      {
         if (definition instanceof FixedJointDefinition)
            return new SimFixedJoint((FixedJointDefinition) definition, predecessor);
         if (definition instanceof PlanarJointDefinition)
            return new SimPlanarJoint((PlanarJointDefinition) definition, predecessor);
         if (definition instanceof SixDoFJointDefinition)
         {
            if (definition.getParentJoint() == null)
               return new SimFloatingRootJoint((SixDoFJointDefinition) definition, predecessor); // TODO Find better way to identify root joint
            else
               return new SimSixDoFJoint((SixDoFJointDefinition) definition, predecessor);
         }
         if (definition instanceof PrismaticJointDefinition)
            return new SimPrismaticJoint((PrismaticJointDefinition) definition, predecessor);
         if (definition instanceof RevoluteJointDefinition)
            return new SimRevoluteJoint((RevoluteJointDefinition) definition, predecessor);
         if (definition instanceof SphericalJointDefinition)
            return new SimSphericalJoint((SphericalJointDefinition) definition, predecessor);
         if (definition instanceof CrossFourBarJointDefinition)
            return new SimCrossFourBarJoint((CrossFourBarJointDefinition) definition, predecessor);
         if (definition instanceof RevoluteTwinsJointDefinition)
            return new SimRevoluteTwinsJoint((RevoluteTwinsJointDefinition) definition, predecessor);
         throw new UnsupportedOperationException("Unsupported joint definition: " + definition.getClass().getSimpleName());
      }
   }

   public static interface RigidBodyBuilderFromDefinition
   {
      default SimRigidBody rootFromDefinition(RigidBodyDefinition rootBodyDefinition, ReferenceFrame inertialFrame, YoRegistry registry)
      {
         return new SimRigidBody(rootBodyDefinition, inertialFrame, registry);
      }

      default SimRigidBody fromDefinition(RigidBodyDefinition rigidBodyDefinition, SimJointBasics parentJoint)
      {
         return new SimRigidBody(rigidBodyDefinition, parentJoint);
      }
   }
}
