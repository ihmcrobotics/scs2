package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlTransient;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Predicate;

/**
 * Definition for a robot. A robot is a tree structure of {@link RigidBodyDefinition} connected by
 * {@link JointDefinition}.
 * <p>
 * A robot definition can be used to create a Robot object that can be simulated or visualized.
 * </p>
 */
@XmlRootElement(name = "Robot")
public class RobotDefinition
{
   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Default implementation for creating a {@link JointBasics} from a {@link JointDefinition}.
    * </p>
    */
   public static final JointCreator DEFAULT_JOINT_BUILDER = (predecessor, definition) -> definition.toJoint(predecessor);
   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Default implementation for creating a {@link RigidBodyBasics} from a {@link RigidBodyDefinition} that will be used as the root body.
    * </p>
    */
   public static final RootBodyCreator DEFAULT_ROOT_BODY_BUILDER = (rootFrame, definition) -> definition.toRootBody(rootFrame);
   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Default implementation for creating a {@link RigidBodyBasics} from a {@link RigidBodyDefinition}.
    * </p>
    */
   public static final RigidBodyCreator DEFAULT_RIGID_BODY_BUILDER = (parentJoint, definition) -> definition.toRigidBody(parentJoint);

   /**
    * The name for the robot.
    */
   private String name;
   /**
    * The root body of the robot.
    * <p>
    * The root body is the first body of the robot, it is the body that is attached to the world. The robot kinematics is defined by the tree of bodies and
    * joints starting from the root body.
    * </p>
    */
   private RigidBodyDefinition rootBodyDefinition;
   /**
    * The names of the joints to ignore in either the controller or the simulation.
    */
   private List<String> nameOfJointsToIgnore = new ArrayList<>();
   /**
    * EXPERIMENTAL: The controllers to be used with this robot.
    */
   private final List<ControllerDefinition> controllerDefinitions = new ArrayList<>();
   /**
    * The class loader used to load the resources associated with this robot.
    * <p>
    * Typically used to load mesh files for the visual and collision shapes.
    * </p>
    */
   private ClassLoader resourceClassLoader;

   /**
    * Creates an empty robot definition. Typically used for XML marshalling.
    */
   public RobotDefinition()
   {
   }

   /**
    * Creates and initializes a robot definition.
    *
    * @param name the name for the robot.
    */
   public RobotDefinition(String name)
   {
      setName(name);
   }

   /**
    * Copy constructor.
    *
    * @param other the other robot definition to copy. Not modified.
    */
   public RobotDefinition(RobotDefinition other)
   {
      name = other.name;
      rootBodyDefinition = other.rootBodyDefinition == null ? null : other.rootBodyDefinition.copyRecursive();
      for (JointDefinition jointDefinition : getAllJoints())
      {
         if (!jointDefinition.isLoopClosure())
            continue;
         String successorName = other.getJointDefinition(jointDefinition.getName()).getSuccessor().getName();
         RigidBodyDefinition rigidBodyDefinition = getRigidBodyDefinition(successorName);
         jointDefinition.setLoopClosureSuccessor(rigidBodyDefinition);
      }
      nameOfJointsToIgnore.addAll(other.nameOfJointsToIgnore);
      resourceClassLoader = other.resourceClassLoader;
   }

   /**
    * Sets the name for this robot.
    *
    * @param name the name for this robot.
    */
   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Sets the root body for this robot.
    * <p>
    * The root body is the first body of the robot, it is the body that is attached to the world. The robot kinematics is defined by the tree of bodies and
    * joints starting from the root body.
    * </p>
    *
    * @param rootBodyDefinition the root body for this robot.
    */
   @XmlElement(name = "rootBody")
   public void setRootBodyDefinition(RigidBodyDefinition rootBodyDefinition)
   {
      this.rootBodyDefinition = rootBodyDefinition;
   }

   /**
    * Specifies that the robot should not be simulated. This is particularly useful for controlling the
    * robot state from outside the session, for instance for visualizing some output as robot state.
    */
   public void ignoreAllJoints()
   {
      for (JointDefinition jointDefinition : collectSubtreeJointDefinitions(rootBodyDefinition))
      {
         addJointToIgnore(jointDefinition.getName());
      }
   }

   /**
    * Specifies the joints that should not be simulated/controlled.
    * <p>
    * Typically used to ignore the finger joints of a humanoid robot that are numerous and hve a negligible effect on the robot's dynamics.
    * </p>
    *
    * @param nameOfJointsToIgnore the name of the joints to ignore.
    */
   @XmlElement(name = "jointToIgnore")
   public void setNameOfJointsToIgnore(List<String> nameOfJointsToIgnore)
   {
      this.nameOfJointsToIgnore = nameOfJointsToIgnore;
   }

   /**
    * Specifies the joints that should not be simulated/controlled. This method adds the given joint to the list of joints to ignore.
    * <p>
    * Typically used to ignore the finger joints of a humanoid robot that are numerous and hve a negligible effect on the robot's dynamics.
    * </p>
    *
    * @param nameOfJointToIgnore the name of the joints to ignore.
    */
   public void addJointToIgnore(String nameOfJointToIgnore)
   {
      nameOfJointsToIgnore.add(nameOfJointToIgnore);
   }

   /**
    * Specifies the joints that should not be simulated/controlled. This method adds the given joint's subtree to the list of joints to ignore.
    * <p>
    * Typically used to ignore the finger joints of a humanoid robot that are numerous and hve a negligible effect on the robot's dynamics.
    * </p>
    *
    * @param nameOfLastJointToConsider the name of the last joint to consider, the subtree starting from this joint will be added to the list of joints to
    *                                  ignore.
    */
   public void addSubtreeJointsToIgnore(String nameOfLastJointToConsider)
   {
      List<JointDefinition> definitionOfJointsToIgnore = collectSubtreeJointDefinitions(getJointDefinition(nameOfLastJointToConsider).getSuccessor());

      for (JointDefinition jointDefinition : definitionOfJointsToIgnore)
      {
         nameOfJointsToIgnore.add(jointDefinition.getName());
      }
   }

   /**
    * Adds a controller to this robot.
    *
    * @param controllerDefinition the controller to add.
    */
   public void addControllerDefinition(ControllerDefinition controllerDefinition)
   {
      controllerDefinitions.add(controllerDefinition);
   }

   /**
    * Simplifies this robot kinematics by removing any fixed joint, i.e. joints with 0 degree of
    * freedom.
    *
    * @see #simplifyKinematics(JointDefinition, Predicate)
    */
   public void simplifyKinematics()
   {
      simplifyKinematics(null);
   }

   /**
    * Simplifies this robot kinematics by removing any fixed joint, i.e. joints with 0 degree of
    * freedom.
    *
    * @param simplifyKinematicsFilter filter that allows to preserve some fixed joints during this
    *                                 operation. A fixed joint is preserved only if the filter is
    *                                 defined and returns {@code false} when queried.
    * @see #simplifyKinematics(JointDefinition, Predicate)
    */
   public void simplifyKinematics(Predicate<FixedJointDefinition> simplifyKinematicsFilter)
   {
      for (int i = 0; i < rootBodyDefinition.getChildrenJoints().size(); i++)
         simplifyKinematics(rootBodyDefinition.getChildrenJoints().get(i), simplifyKinematicsFilter);
   }

   /**
    * Transforms all the robot local frames such that they are pointing z-up and x-forward when the
    * robot is in the zero configuration.
    *
    * @see #transformAllFramesToZUp(JointDefinition)
    */
   public void transformAllFramesToZUp()
   {
      for (int i = 0; i < rootBodyDefinition.getChildrenJoints().size(); i++)
         transformAllFramesToZUp(rootBodyDefinition.getChildrenJoints().get(i));
   }

   /**
    * Sanitizes the names of the joints and rigid-bodies by replacing all dots '.' with underscores
    * '_'.
    */
   public void sanitizeNames()
   {
      forEachJointDefinition(jointDefinition ->
                             {
                                String sanitizedName = jointDefinition.getName();
                                sanitizedName = sanitizedName.replace('.', '_');
                                jointDefinition.setName(sanitizedName);
                             });
      forEachRigidBodyDefinition(rigidBodyDefinition ->
                                 {
                                    String sanitizedName = rigidBodyDefinition.getName();
                                    sanitizedName = sanitizedName.replace('.', '_');
                                    rigidBodyDefinition.setName(sanitizedName);
                                 });
   }

   /**
    * Sets the class loader used to load the resources associated with this robot.
    * <p>
    * Typically used to load mesh files for the visual and collision shapes.
    * </p>
    *
    * @param resourceClassLoader the class loader used to load the resources associated with this robot.
    */
   @XmlTransient
   public void setResourceClassLoader(ClassLoader resourceClassLoader)
   {
      this.resourceClassLoader = resourceClassLoader;
   }

   /**
    * Returns the name for this robot.
    *
    * @return the name for this robot.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Returns the root body for this robot.
    * <p>
    * The root body is the first body of the robot, it is the body that is attached to the world. The robot kinematics is defined by the tree of bodies and
    * joints starting from the root body.
    * </p>
    *
    * @return the root body for this robot.
    */
   public RigidBodyDefinition getRootBodyDefinition()
   {
      return rootBodyDefinition;
   }

   /**
    * Returns the floating root joint definition for this robot.
    * <p>
    * The floating root joint is the first joint of the robot, it is the joint that is attached to the world. The robot kinematics is defined by the tree of
    * bodies and joints starting from the floating root joint.
    * </p>
    *
    * @return the floating root joint definition for this robot.
    */
   public SixDoFJointDefinition getFloatingRootJointDefinition()
   {
      if (rootBodyDefinition == null)
         return null;
      if (rootBodyDefinition.getChildrenJoints().size() != 1)
         return null;

      JointDefinition jointDefinition = getRootJointDefinitions().get(0);

      if (jointDefinition instanceof SixDoFJointDefinition)
         return (SixDoFJointDefinition) jointDefinition;

      return null;
   }

   /**
    * Returns the all the root joint definitions for this robot.
    * <p>
    * Typically, there is a single root joint that is of the type {@link SixDoFJointDefinition}.
    * </p>
    *
    * @return the all the root joint definitions for this robot.
    */
   public List<JointDefinition> getRootJointDefinitions()
   {
      return rootBodyDefinition.getChildrenJoints();
   }

   /**
    * Returns the names of the joints to ignore in either the controller or the simulation.
    *
    * @return the names of the joints to ignore in either the controller or the simulation.
    */
   public List<String> getNameOfJointsToIgnore()
   {
      return nameOfJointsToIgnore;
   }

   /**
    * Returns the class loader used to load the resources associated with this robot.
    * <p>
    * Typically used to load mesh files for the visual and collision shapes.
    * </p>
    *
    * @return the class loader used to load the resources associated with this robot.
    */
   public ClassLoader getResourceClassLoader()
   {
      return resourceClassLoader;
   }

   /**
    * Returns the joint definition for the given joint name.
    *
    * @param jointName the name of the joint to retrieve.
    * @return the joint definition for the given joint name or {@code null} if no joint could be found.
    */
   public JointDefinition getJointDefinition(String jointName)
   {
      return findJointDefinition(rootBodyDefinition, jointName);
   }

   /**
    * Returns the one degree-of-freedom joint definition for the given joint name.
    *
    * @param jointName the name of the joint to retrieve.
    * @return the one degree-of-freedom joint definition for the given joint name or {@code null} if no joint could be found.
    */
   public OneDoFJointDefinition getOneDoFJointDefinition(String jointName)
   {
      JointDefinition jointDefinition = getJointDefinition(jointName);
      if (jointDefinition instanceof OneDoFJointDefinition)
         return (OneDoFJointDefinition) jointDefinition;
      else
         return null;
   }

   /**
    * Returns the rigid-body definition for the given rigid-body name.
    *
    * @param bodyName the name of the rigid-body to retrieve.
    * @return the rigid-body definition for the given rigid-body name or {@code null} if no rigid-body could be found.
    */
   public RigidBodyDefinition getRigidBodyDefinition(String bodyName)
   {
      return findRigidBodyDefinition(rootBodyDefinition, bodyName);
   }

   /**
    * Performs the given action for each joint definition in the robot.
    *
    * @param jointConsumer the action to perform.
    */
   public void forEachJointDefinition(Consumer<JointDefinition> jointConsumer)
   {
      forEachJointDefinition(rootBodyDefinition, jointConsumer);
   }

   /**
    * Performs the given action for each one degree-of-freedom joint definition in the robot.
    *
    * @param jointConsumer the action to perform.
    */
   public void forEachOneDoFJointDefinition(Consumer<OneDoFJointDefinition> jointConsumer)
   {
      forEachJointDefinition(rootBodyDefinition, joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
            jointConsumer.accept((OneDoFJointDefinition) joint);
      });
   }

   /**
    * Performs the given action for each rigid-body definition in the robot.
    *
    * @param rigidBodyConsumer the action to perform.
    */
   public void forEachRigidBodyDefinition(Consumer<RigidBodyDefinition> rigidBodyConsumer)
   {
      forEachRigidBodyDefinition(rootBodyDefinition, rigidBodyConsumer);
   }

   /**
    * Returns the list of all the joint definitions in the robot.
    *
    * @return the list of all the joint definitions in the robot.
    */
   public List<JointDefinition> getAllJoints()
   {
      List<JointDefinition> joints = new ArrayList<>();
      forEachJointDefinition(joints::add);
      return joints;
   }

   /**
    * Returns the list of all the one degree-of-freedom joint definitions in the robot.
    *
    * @return the list of all the one degree-of-freedom joint definitions in the robot.
    */
   public List<OneDoFJointDefinition> getAllOneDoFJoints()
   {
      List<OneDoFJointDefinition> joints = new ArrayList<>();
      forEachOneDoFJointDefinition(joints::add);
      return joints;
   }

   /**
    * Returns the list of all the rigid-body definitions in the robot.
    *
    * @return the list of all the rigid-body definitions in the robot.
    */
   public List<RigidBodyDefinition> getAllRigidBodies()
   {
      return collectSubtreeRigidBodyDefinitions(rootBodyDefinition);
   }

   /**
    * Returns the list of all the controllers for this robot.
    *
    * @return the list of all the controllers for this robot.
    */
   public List<ControllerDefinition> getControllerDefinitions()
   {
      return controllerDefinitions;
   }

   /**
    * Instantiates a new robot with the given root frame.
    *
    * @param rootFrame the root frame for the robot.
    * @return the new robot.
    */
   public RigidBodyBasics newInstance(ReferenceFrame rootFrame)
   {
      return newInstance(rootFrame, DEFAULT_ROOT_BODY_BUILDER, DEFAULT_JOINT_BUILDER, DEFAULT_RIGID_BODY_BUILDER);
   }

   /**
    * Instantiates a new robot with the given root frame.
    *
    * @param rootFrame        the root frame for the robot.
    * @param rootBodyCreator  the creator for the root body.
    * @param jointCreator     the creator for the joints.
    * @param rigidBodyCreator the creator for the rigid-bodies.
    * @return the new robot.
    */
   public RigidBodyBasics newInstance(ReferenceFrame rootFrame, RootBodyCreator rootBodyCreator, JointCreator jointCreator, RigidBodyCreator rigidBodyCreator)
   {
      if (rootBodyDefinition == null)
         throw new NullPointerException("The robot " + name + " has no definition!");
      RigidBodyBasics rootBody = rootBodyCreator.newRootBody(rootFrame, rootBodyDefinition);
      instantiateRecursively(rootBody, rootBodyDefinition, jointCreator, rigidBodyCreator);
      closeLoops(rootBody, rootBodyDefinition);
      return rootBody;
   }

   private void instantiateRecursively(RigidBodyBasics predecessor,
                                       RigidBodyDefinition predecessorDefinition,
                                       JointCreator jointCreator,
                                       RigidBodyCreator rigidBodyCreator)
   {
      for (JointDefinition childDefinition : predecessorDefinition.getChildrenJoints())
      {
         JointBasics child = jointCreator.newJoint(predecessor, childDefinition);
         if (childDefinition.isLoopClosure())
            continue; // The successor will be created by another path and will be attached to the loop closure afterward.
         RigidBodyDefinition successorDefinition = childDefinition.getSuccessor();

         if (successorDefinition == null)
            throw new NullPointerException("The joint " + child.getName() + " is missing the definition for its successor, robot name: " + name + ".");

         RigidBodyBasics successor = rigidBodyCreator.newRigidBody(child, successorDefinition);
         instantiateRecursively(successor, successorDefinition, jointCreator, rigidBodyCreator);
      }
   }

   /**
    * <i>-- Intended for internal use --</i>
    * Recursively closes the loops in the robot.
    *
    * @param predecessor           the predecessor of the subtree to close the loops for.
    * @param predecessorDefinition the definition of the predecessor of the subtree to close the loops for.
    */
   public static void closeLoops(RigidBodyBasics predecessor, RigidBodyDefinition predecessorDefinition)
   {
      if (predecessor == null)
         return;

      for (int i = 0; i < predecessor.getChildrenJoints().size(); i++)
      {
         JointBasics joint = predecessor.getChildrenJoints().get(i);
         JointDefinition jointDefinition = predecessorDefinition.getChildrenJoints().get(i);

         if (jointDefinition.isLoopClosure())
         {
            RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(predecessor);
            RigidBodyBasics successor = MultiBodySystemTools.findRigidBody(rootBody, jointDefinition.getSuccessor().getName());
            RigidBodyTransformReadOnly transformFromSuccessorParentJoint = jointDefinition.getLoopClosureDefinition().getTransformToSuccessorParent();
            joint.setupLoopClosure(successor, transformFromSuccessorParentJoint);
         }

         closeLoops(joint.getSuccessor(), jointDefinition.getSuccessor());
      }
   }

   /**
    * Recursively traverses the robot kinematics and returns the list of all the joints that belong to the {@code start} subtree.
    *
    * @param start the subtree to collect the joints from.
    * @return the list of all the joints that belong to the {@code start} subtree.
    */
   public static List<JointDefinition> collectSubtreeJointDefinitions(RigidBodyDefinition start)
   {
      List<JointDefinition> joints = new ArrayList<>();
      forEachJointDefinition(start, joints::add);
      return joints;
   }

   /**
    * Recursively traverses the robot kinematics and returns the list of all the rigid-bodies that belong to the {@code start} subtree.
    *
    * @param start the subtree to collect the rigid-bodies from.
    * @return the list of all the rigid-bodies that belong to the {@code start} subtree.
    */
   public static List<RigidBodyDefinition> collectSubtreeRigidBodyDefinitions(RigidBodyDefinition start)
   {
      List<RigidBodyDefinition> rigidBodies = new ArrayList<>();
      forEachRigidBodyDefinition(start, rigidBodies::add);
      return rigidBodies;
   }

   /**
    * Searches the robot kinematics for the joint definition with the given name and that is a descendant of {@code start}.
    *
    * @param start     the subtree to start the search from.
    * @param jointName the name of the joint to find.
    * @return the joint definition with the given name or {@code null} if no joint could be found.
    */
   public static JointDefinition findJointDefinition(RigidBodyDefinition start, String jointName)
   {
      if (start == null)
         return null;

      for (int i = 0; i < start.getChildrenJoints().size(); i++)
      {
         JointDefinition jointDefinition = start.getChildrenJoints().get(i);

         if (jointDefinition.getName().equals(jointName))
            return jointDefinition;

         JointDefinition result = findJointDefinition(jointDefinition.getSuccessor(), jointName);

         if (result != null)
            return result;
      }

      return null;
   }

   /**
    * Assuming the given {@code robotDefinition} and {@code rootBody} are compatible, this method recurse through the robot kinematics and initializes the
    * robot state to the initial state specified in the robot definition.
    *
    * @param robotDefinition the definition with initial state.
    * @param rootBody        the root body of the robot to initialize the state of.
    */
   public static void initializeRobotState(RobotDefinition robotDefinition, RigidBodyBasics rootBody)
   {
      initializeRobotStateRecursive(robotDefinition.getRootBodyDefinition(), rootBody);
   }

   private static void initializeRobotStateRecursive(RigidBodyDefinition definition, RigidBodyBasics rigidBody)
   {
      if (definition.getChildrenJoints().size() != rigidBody.getChildrenJoints().size())
         throw new IllegalArgumentException("Robot mismatch at rigid-body: " + definition.getName());

      for (int i = 0; i < definition.getChildrenJoints().size(); i++)
      {
         JointDefinition jointDefinition = definition.getChildrenJoints().get(i);
         JointBasics joint = rigidBody.getChildrenJoints().get(i);

         if (!jointDefinition.getName().equals(joint.getName()))
            throw new IllegalArgumentException("Definition incompatible with robot. Expected joint: " + definition.getName() + ", was: " + joint.getName());

         JointStateBasics initialJointState = jointDefinition.getInitialJointState();

         if (initialJointState != null)
         {
            if (initialJointState.hasOutputFor(JointStateType.CONFIGURATION))
               initialJointState.getConfiguration(joint);
            if (initialJointState.hasOutputFor(JointStateType.VELOCITY))
               initialJointState.getVelocity(joint);
            if (initialJointState.hasOutputFor(JointStateType.ACCELERATION))
               initialJointState.getAcceleration(joint);
            if (initialJointState.hasOutputFor(JointStateType.EFFORT))
               initialJointState.getEffort(joint);
         }

         initializeRobotStateRecursive(jointDefinition.getSuccessor(), joint.getSuccessor());
      }
   }

   /**
    * Searches the robot kinematics for the rigid-body definition with the given name and that is a descendant of {@code start}.
    *
    * @param start         the subtree to start the search from.
    * @param rigidBodyName the name of the rigid-body to find.
    * @return the rigid-body definition with the given name or {@code null} if no rigid-body could be found.
    */
   public static RigidBodyDefinition findRigidBodyDefinition(RigidBodyDefinition start, String rigidBodyName)
   {
      if (start == null)
         return null;
      if (start.getName().equals(rigidBodyName))
         return start;

      for (int i = 0; i < start.getChildrenJoints().size(); i++)
      {
         RigidBodyDefinition result = findRigidBodyDefinition(start.getChildrenJoints().get(i).getSuccessor(), rigidBodyName);

         if (result != null)
            return result;
      }
      return null;
   }

   /**
    * Performs the given action for each joint definition in the robot.
    *
    * @param start         the subtree to start performing the action from.
    * @param jointConsumer the action to perform.
    */
   public static void forEachJointDefinition(RigidBodyDefinition start, Consumer<JointDefinition> jointConsumer)
   {
      if (start == null)
         return;

      for (int i = 0; i < start.getChildrenJoints().size(); i++)
      {
         JointDefinition jointDefinition = start.getChildrenJoints().get(i);
         jointConsumer.accept(jointDefinition);
         forEachJointDefinition(jointDefinition.getSuccessor(), jointConsumer);
      }
   }

   /**
    * Performs the given action for each rigid-body definition in the robot.
    *
    * @param start             the subtree to start performing the action from.
    * @param rigidBodyConsumer the action to perform.
    */
   public static void forEachRigidBodyDefinition(RigidBodyDefinition start, Consumer<RigidBodyDefinition> rigidBodyConsumer)
   {
      if (start == null)
         return;

      rigidBodyConsumer.accept(start);

      for (int i = 0; i < start.getChildrenJoints().size(); i++)
      {
         forEachRigidBodyDefinition(start.getChildrenJoints().get(i).getSuccessor(), rigidBodyConsumer);
      }
   }

   /**
    * Interface used to create a {@link JointBasics} from a {@link JointDefinition}.
    */
   public interface JointCreator
   {
      /**
       * Creates a new joint from the given {@code definition} and attaches it to the given
       * {@code predecessor}.
       *
       * @param predecessor the predecessor of the new joint.
       * @param definition  the definition of the new joint.
       * @return the new joint.
       */
      JointBasics newJoint(RigidBodyBasics predecessor, JointDefinition definition);
   }

   /**
    * Interface used to create a {@link RigidBodyBasics} from a {@link RigidBodyDefinition}.
    */
   public interface RigidBodyCreator
   {
      /**
       * Creates a new rigid-body from the given {@code definition} and attaches it to the given
       * {@code parentJoint}.
       *
       * @param parentJoint the parent joint of the new rigid-body.
       * @param definition  the definition of the new rigid-body.
       * @return the new rigid-body.
       */
      RigidBodyBasics newRigidBody(JointBasics parentJoint, RigidBodyDefinition definition);
   }

   /**
    * Interface used to create a {@link RigidBodyBasics} from a {@link RigidBodyDefinition} that will be used as the root body.
    */
   public interface RootBodyCreator
   {
      /**
       * Creates a new root body from the given {@code definition} and attaches it to the given
       * {@code parentFrame}.
       *
       * @param parentFrame the parent frame of the new root body.
       * @param definition  the definition of the new root body.
       * @return the new root body.
       */
      RigidBodyBasics newRootBody(ReferenceFrame parentFrame, RigidBodyDefinition definition);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, name);
      bits = EuclidHashCodeTools.addToHashCode(bits, rootBodyDefinition);
      bits = EuclidHashCodeTools.addToHashCode(bits, nameOfJointsToIgnore);
      bits = EuclidHashCodeTools.addToHashCode(bits, controllerDefinitions);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      RobotDefinition other = (RobotDefinition) object;

      if (!Objects.equals(name, other.name))
         return false;
      if (!Objects.equals(rootBodyDefinition, other.rootBodyDefinition))
         return false;
      if (!Objects.equals(nameOfJointsToIgnore, other.nameOfJointsToIgnore))
         return false;
      if (!Objects.equals(controllerDefinitions, other.controllerDefinitions))
         return false;

      return true;
   }

   /**
    * Recursive method that performs the following modifications will preserving the robot's physical
    * qualities:
    * <ul>
    * <li>adjust orientations such that the joint poses are z-up when the robot is at the zero joint
    * configuration.
    * <li>transform the moment of inertia for all rigid-body such that their inertia pose is only a
    * translation.
    * </ul>
    *
    * @param jointDefinition starting point for the recursion.
    */
   public static void transformAllFramesToZUp(JointDefinition jointDefinition)
   {
      transformAllFramesToZUp(jointDefinition, new RigidBodyTransform(jointDefinition.getTransformToParent()));
   }

   // Switching the transform type for the joint definition from YawPitchRollDefinition to RigidBodyTransform to avoid gimbal lock when concatenating rotations.
   private static void transformAllFramesToZUp(JointDefinition jointDefinition, RigidBodyTransformReadOnly jointTransform)
   {
      Orientation3DReadOnly jointRotation = jointTransform.getRotation();
      if (jointDefinition instanceof OneDoFJointDefinition)
         jointRotation.transform(((OneDoFJointDefinition) jointDefinition).getAxis());
      RigidBodyDefinition linkDefinition = jointDefinition.getSuccessor();
      RigidBodyTransformBasics inertiaPose = new RigidBodyTransform(linkDefinition.getInertiaPose());
      inertiaPose.prependOrientation(jointRotation);
      Matrix3D momentOfInertia = new Matrix3D(linkDefinition.getMomentOfInertia());
      inertiaPose.transform(momentOfInertia);
      linkDefinition.getMomentOfInertia().set(momentOfInertia);
      linkDefinition.getInertiaPose().getRotation().setToZero();
      linkDefinition.getInertiaPose().getTranslation().set(inertiaPose.getTranslation());

      for (KinematicPointDefinition kinematicPointDefinition : jointDefinition.getKinematicPointDefinitions())
         kinematicPointDefinition.getTransformToParent().prependOrientation(jointRotation);
      for (ExternalWrenchPointDefinition externalWrenchPointDefinition : jointDefinition.getExternalWrenchPointDefinitions())
         externalWrenchPointDefinition.getTransformToParent().prependOrientation(jointRotation);
      for (GroundContactPointDefinition groundContactPointDefinition : jointDefinition.getGroundContactPointDefinitions())
         groundContactPointDefinition.getTransformToParent().prependOrientation(jointRotation);

      for (SensorDefinition sensorDefinition : jointDefinition.getSensorDefinitions())
         sensorDefinition.getTransformToJoint().prependOrientation(jointRotation);

      for (VisualDefinition visualDefinition : linkDefinition.getVisualDefinitions())
         visualDefinition.getOriginPose().prependOrientation(jointRotation);

      for (CollisionShapeDefinition collisionShapeDefinition : linkDefinition.getCollisionShapeDefinitions())
         collisionShapeDefinition.getOriginPose().prependOrientation(jointRotation);

      for (JointDefinition childDefinition : jointDefinition.getSuccessor().getChildrenJoints())
      {
         RigidBodyTransform childTransform = new RigidBodyTransform(childDefinition.getTransformToParent());
         childTransform.prependOrientation(jointRotation);
         childDefinition.getTransformToParent().set(childTransform);
         transformAllFramesToZUp(childDefinition, childTransform);
      }

      jointDefinition.getTransformToParent().getRotation().setToZero();
   }

   /**
    * Navigates the subtree starting from the given joint and simplifies the kinematics by removing all
    * {@link FixedJointDefinition}.
    * <p>
    * Whenever a {@code FixedJointDefinition} is removed, the following operations are performed:
    * <ul>
    * <li>the fixed joint successor's physical properties (mass, inertia) and visuals are combined to
    * the joint's predecessor.
    * <li>the fixed joint sensors are moved to the parent joint. The move includes adjusting the pose
    * of each sensor so they remain at the same physical location on the robot.
    * </ul>
    * </p>
    *
    * @param joint  the first joint from which to simplify the kinematics.
    * @param filter a fixed joint is only removed if: the filter is {@code null} or
    *               {@code filter.test(joint)} is {@code true}. If a filter is provided, any fixed
    *               joint for which it returns {@code false} will <b>not</b> be removed.
    */
   public static void simplifyKinematics(JointDefinition joint, Predicate<FixedJointDefinition> filter)
   {
      // The children list may shrink or grow depending the simplyKinematics(joint.child)
      // Also, if a child is a fixed-joint, the successor of this joint will be replaced with a new one, so can't save the successor as a local variable.
      for (int i = 0; i < joint.getSuccessor().getChildrenJoints().size(); )
      {
         List<JointDefinition> children = joint.getSuccessor().getChildrenJoints();
         JointDefinition child = children.get(i);

         if (!(child instanceof FixedJointDefinition))
            i++; // This child won't be removed, we can increment to the next.

         simplifyKinematics(child, filter);
      }

      JointDefinition parentJoint = joint.getParentJoint();
      if (parentJoint == null)
         return;

      if (joint instanceof FixedJointDefinition fixedJoint && (filter == null || filter.test(fixedJoint)))
      {
         RigidBodyDefinition rigidBody = joint.getSuccessor();
         YawPitchRollTransformDefinition transformToParentJoint = joint.getTransformToParent();

         rigidBody.applyTransform(transformToParentJoint);
         RigidBodyDefinition oldParentRigidBody = parentJoint.getSuccessor();
         parentJoint.setSuccessor(merge(oldParentRigidBody.getName(), oldParentRigidBody, rigidBody));
         parentJoint.getSuccessor().addChildJoints(oldParentRigidBody.getChildrenJoints());

         joint.getKinematicPointDefinitions().removeIf(kp ->
                                                       {
                                                          kp.applyTransform(transformToParentJoint);
                                                          parentJoint.addKinematicPointDefinition(kp);
                                                          return true;
                                                       });
         joint.getExternalWrenchPointDefinitions().removeIf(efp ->
                                                            {
                                                               efp.applyTransform(transformToParentJoint);
                                                               parentJoint.addExternalWrenchPointDefinition(efp);
                                                               return true;
                                                            });
         joint.getGroundContactPointDefinitions().removeIf(gcp ->
                                                           {
                                                              gcp.applyTransform(transformToParentJoint);
                                                              parentJoint.addGroundContactPointDefinition(gcp);
                                                              return true;
                                                           });
         joint.getSensorDefinitions().removeIf(sensor ->
                                               {
                                                  sensor.applyTransform(transformToParentJoint);
                                                  parentJoint.addSensorDefinition(sensor);
                                                  return true;
                                               });
         joint.getSuccessor().getChildrenJoints().removeIf(child ->
                                                           {
                                                              child.getTransformToParent().preMultiply(transformToParentJoint);
                                                              parentJoint.getSuccessor().addChildJoint(child);
                                                              return true;
                                                           });
         parentJoint.getSuccessor().removeChildJoint(joint);
      }
   }

   /**
    * <i>-- Intended for internal use --</i>
    * <p>
    * Creates a new rigid-body which physical properties equals the sum of {@code rigidBodyA} and
    * {@code rigidBody}. In addition, the visuals and collisions are added to the merged body.
    * </p>
    * <p>
    * Note the following property:
    * {@code merge("bodyAB", bodyA, bodyB) == merge("bodyAB", bodyB, bodyA)}.
    * </p>
    *
    * @param name       the name of the merged rigid-body.
    * @param rigidBodyA the first rigid-body to merge.
    * @param rigidBodyB the second rigid-body to merge.
    * @return the merged body.
    */
   public static RigidBodyDefinition merge(String name, RigidBodyDefinition rigidBodyA, RigidBodyDefinition rigidBodyB)
   {
      double mergedMass = rigidBodyA.getMass() + rigidBodyB.getMass();
      Vector3D mergedCoM = new Vector3D();
      mergedCoM.setAndScale(rigidBodyA.getMass(), rigidBodyA.getCenterOfMassOffset());
      mergedCoM.scaleAdd(rigidBodyB.getMass(), rigidBodyB.getCenterOfMassOffset(), mergedCoM);
      mergedCoM.scale(1.0 / mergedMass);

      Vector3D translationInertiaA = new Vector3D();
      translationInertiaA.sub(mergedCoM, rigidBodyA.getCenterOfMassOffset());
      Matrix3D inertiaA = new Matrix3D(rigidBodyA.getMomentOfInertia());
      MecanoTools.translateMomentOfInertia(rigidBodyA.getMass(), null, false, translationInertiaA, inertiaA);

      Vector3D translationInertiaB = new Vector3D();
      translationInertiaB.sub(mergedCoM, rigidBodyB.getCenterOfMassOffset());
      Matrix3D inertiaB = new Matrix3D(rigidBodyB.getMomentOfInertia());
      MecanoTools.translateMomentOfInertia(rigidBodyB.getMass(), null, false, translationInertiaB, inertiaB);

      Matrix3D mergedInertia = new Matrix3D();
      mergedInertia.add(inertiaA);
      mergedInertia.add(inertiaB);

      RigidBodyDefinition merged = new RigidBodyDefinition(name);
      merged.setMass(mergedMass);
      merged.getInertiaPose().getTranslation().set(mergedCoM);
      merged.getMomentOfInertia().set(mergedInertia);

      List<VisualDefinition> mergedGraphics = new ArrayList<>();
      mergedGraphics.addAll(rigidBodyA.getVisualDefinitions());
      mergedGraphics.addAll(rigidBodyB.getVisualDefinitions());
      merged.addVisualDefinitions(mergedGraphics);

      List<CollisionShapeDefinition> mergedCollisions = new ArrayList<>();
      mergedCollisions.addAll(rigidBodyA.getCollisionShapeDefinitions());
      mergedCollisions.addAll(rigidBodyB.getCollisionShapeDefinitions());
      merged.addCollisionShapeDefinitions(mergedCollisions);

      return merged;
   }
}
