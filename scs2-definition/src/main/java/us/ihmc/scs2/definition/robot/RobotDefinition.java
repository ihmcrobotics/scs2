package us.ihmc.scs2.definition.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlTransient;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;

@XmlRootElement(name = "Robot")
public class RobotDefinition
{
   public static final JointCreator DEFAULT_JOINT_BUILDER = (predecessor, definition) -> definition.toJoint(predecessor);
   public static final RootBodyCreator DEFAUL_ROOT_BODY_BUILDER = (rootFrame, definition) -> definition.toRootBody(rootFrame);
   public static final RigidBodyCreator DEFAUL_RIGID_BODY_BUILDER = (parentJoint, definition) -> definition.toRigidBody(parentJoint);

   private String name;
   private RigidBodyDefinition rootBodyDefinition;
   private List<String> nameOfJointsToIgnore = new ArrayList<>();

   private final List<ControllerDefinition> controllerDefinitions = new ArrayList<>();

   private ClassLoader resourceClassLoader;

   public RobotDefinition()
   {
   }

   public RobotDefinition(String name)
   {
      setName(name);
   }

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

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement(name = "rootBody")
   public void setRootBodyDefinition(RigidBodyDefinition rootBodyDefinition)
   {
      this.rootBodyDefinition = rootBodyDefinition;
   }

   public void ignoreAllJoints()
   {
      for (JointDefinition jointDefinition : collectSubtreeJointDefinitions(rootBodyDefinition))
      {
         addJointToIgnore(jointDefinition.getName());
      }
   }

   @XmlElement(name = "jointToIgnore")
   public void setNameOfJointsToIgnore(List<String> nameOfJointsToIgnore)
   {
      this.nameOfJointsToIgnore = nameOfJointsToIgnore;
   }

   public void addJointToIgnore(String nameOfJointToIgnore)
   {
      nameOfJointsToIgnore.add(nameOfJointToIgnore);
   }

   public void addSubtreeJointsToIgnore(String nameOfLastJointToConsider)
   {
      List<JointDefinition> definitionOfJointsToIgnore = collectSubtreeJointDefinitions(getJointDefinition(nameOfLastJointToConsider).getSuccessor());

      for (JointDefinition jointDefinition : definitionOfJointsToIgnore)
      {
         nameOfJointsToIgnore.add(jointDefinition.getName());
      }
   }

   public void addControllerDefinition(ControllerDefinition controllerDefinition)
   {
      controllerDefinitions.add(controllerDefinition);
   }

   @XmlTransient
   public void setResourceClassLoader(ClassLoader resourceClassLoader)
   {
      this.resourceClassLoader = resourceClassLoader;
   }

   public String getName()
   {
      return name;
   }

   public RigidBodyDefinition getRootBodyDefinition()
   {
      return rootBodyDefinition;
   }

   public List<JointDefinition> getRootJointDefinitions()
   {
      return rootBodyDefinition.getChildrenJoints();
   }

   public List<String> getNameOfJointsToIgnore()
   {
      return nameOfJointsToIgnore;
   }

   public ClassLoader getResourceClassLoader()
   {
      return resourceClassLoader;
   }

   public JointDefinition getJointDefinition(String jointName)
   {
      return findJointDefinition(rootBodyDefinition, jointName);
   }

   public OneDoFJointDefinition getOneDoFJointDefinition(String jointName)
   {
      JointDefinition jointDefinition = getJointDefinition(jointName);
      if (jointDefinition instanceof OneDoFJointDefinition)
         return (OneDoFJointDefinition) jointDefinition;
      else
         return null;
   }

   public RigidBodyDefinition getRigidBodyDefinition(String bodyName)
   {
      return findRigidBodyDefinition(rootBodyDefinition, bodyName);
   }

   public void forEachJointDefinition(Consumer<JointDefinition> jointConsumer)
   {
      forEachJointDefinition(rootBodyDefinition, jointConsumer);
   }

   public void forEachOneDoFJointDefinition(Consumer<OneDoFJointDefinition> jointConsumer)
   {
      forEachJointDefinition(rootBodyDefinition, joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
            jointConsumer.accept((OneDoFJointDefinition) joint);
      });
   }

   public void forEachRigidBodyDefinition(Consumer<RigidBodyDefinition> rigidBodyConsumer)
   {
      forEachRigidBodyDefinition(rootBodyDefinition, rigidBodyConsumer);
   }

   public List<JointDefinition> getAllJoints()
   {
      List<JointDefinition> joints = new ArrayList<>();
      forEachJointDefinition(joints::add);
      return joints;
   }

   public List<OneDoFJointDefinition> getAllOneDoFJoints()
   {
      List<OneDoFJointDefinition> joints = new ArrayList<>();
      forEachOneDoFJointDefinition(joints::add);
      return joints;
   }

   public List<RigidBodyDefinition> getAllRigidBodies()
   {
      return collectSubtreeRigidBodyDefinitions(rootBodyDefinition);
   }

   public List<ControllerDefinition> getControllerDefinitions()
   {
      return controllerDefinitions;
   }

   public RigidBodyBasics newInstance(ReferenceFrame rootFrame)
   {
      return newInstance(rootFrame, DEFAUL_ROOT_BODY_BUILDER, DEFAULT_JOINT_BUILDER, DEFAUL_RIGID_BODY_BUILDER);
   }

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

   public static List<JointDefinition> collectSubtreeJointDefinitions(RigidBodyDefinition start)
   {
      List<JointDefinition> joints = new ArrayList<>();
      forEachJointDefinition(start, joints::add);
      return joints;
   }

   public static List<RigidBodyDefinition> collectSubtreeRigidBodyDefinitions(RigidBodyDefinition start)
   {
      List<RigidBodyDefinition> rigidBodies = new ArrayList<>();
      forEachRigidBodyDefinition(start, rigidBodies::add);
      return rigidBodies;
   }

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

   public static interface JointCreator
   {
      JointBasics newJoint(RigidBodyBasics predecessor, JointDefinition definition);
   }

   public static interface RigidBodyCreator
   {
      RigidBodyBasics newRigidBody(JointBasics parentJoint, RigidBodyDefinition definition);
   }

   public static interface RootBodyCreator
   {
      RigidBodyBasics newRootBody(ReferenceFrame parentFrame, RigidBodyDefinition definition);
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((controllerDefinitions == null) ? 0 : controllerDefinitions.hashCode());
      result = prime * result + ((name == null) ? 0 : name.hashCode());
      result = prime * result + ((nameOfJointsToIgnore == null) ? 0 : nameOfJointsToIgnore.hashCode());
      result = prime * result + ((rootBodyDefinition == null) ? 0 : rootBodyDefinition.hashCode());
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      RobotDefinition other = (RobotDefinition) obj;
      if (controllerDefinitions == null)
      {
         if (other.controllerDefinitions != null)
            return false;
      }
      else if (!controllerDefinitions.equals(other.controllerDefinitions))
         return false;
      if (name == null)
      {
         if (other.name != null)
            return false;
      }
      else if (!name.equals(other.name))
         return false;
      if (nameOfJointsToIgnore == null)
      {
         if (other.nameOfJointsToIgnore != null)
            return false;
      }
      else if (!nameOfJointsToIgnore.equals(other.nameOfJointsToIgnore))
         return false;
      if (rootBodyDefinition == null)
      {
         if (other.rootBodyDefinition != null)
            return false;
      }
      else if (!rootBodyDefinition.equals(other.rootBodyDefinition))
         return false;
      return true;
   }
}
