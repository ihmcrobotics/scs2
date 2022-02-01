package us.ihmc.scs2.definition.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlTransient;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public abstract class JointDefinition implements Transformable
{
   private String name;
   private YawPitchRollTransformDefinition transformToParent = new YawPitchRollTransformDefinition();

   private RigidBodyDefinition predecessor;
   private RigidBodyDefinition successor;

   private List<SensorDefinition> sensorDefinitions = new ArrayList<>();
   private List<KinematicPointDefinition> kinematicPointDefinitions = new ArrayList<>();
   private List<ExternalWrenchPointDefinition> externalWrenchPointDefinitions = new ArrayList<>();
   private List<GroundContactPointDefinition> groundContactPointDefinitions = new ArrayList<>();

   private LoopClosureDefinition loopClosureDefinition = null;

   public JointDefinition()
   {
   }

   public JointDefinition(String name)
   {
      setName(name);
   }

   public JointDefinition(String name, Tuple3DReadOnly offsetFromParent)
   {
      this(name);
      transformToParent.getTranslation().set(offsetFromParent);
   }

   public JointDefinition(JointDefinition other)
   {
      name = other.name;
      transformToParent.set(other.transformToParent);
      for (SensorDefinition sensorDefinition : other.sensorDefinitions)
         sensorDefinitions.add(sensorDefinition.copy());
      for (KinematicPointDefinition kinematicPointDefinition : other.kinematicPointDefinitions)
         kinematicPointDefinitions.add(kinematicPointDefinition.copy());
      for (ExternalWrenchPointDefinition externalWrenchPointDefinition : other.externalWrenchPointDefinitions)
         externalWrenchPointDefinitions.add(externalWrenchPointDefinition.copy());
      for (GroundContactPointDefinition groundContactPointDefinition : other.groundContactPointDefinitions)
         groundContactPointDefinitions.add(groundContactPointDefinition.copy());
      loopClosureDefinition = other.loopClosureDefinition == null ? null : other.loopClosureDefinition.copy();
   }

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   @XmlElement
   public void setTransformToParent(YawPitchRollTransformDefinition transformToParent)
   {
      this.transformToParent = transformToParent;
   }

   public void setTransformToParent(RigidBodyTransformReadOnly transformToParent)
   {
      this.transformToParent.set(transformToParent);
   }

   public YawPitchRollTransformDefinition getTransformToParent()
   {
      return transformToParent;
   }

   @XmlTransient
   public void setPredecessor(RigidBodyDefinition predecessor)
   {
      this.predecessor = predecessor;
   }

   public RigidBodyDefinition getPredecessor()
   {
      return predecessor;
   }

   @XmlTransient
   public void setLoopClosureSuccessor(RigidBodyDefinition successor)
   {
      if (loopClosureDefinition == null)
         loopClosureDefinition = new LoopClosureDefinition();
      this.successor = successor;
   }

   @XmlElement
   public void setSuccessor(RigidBodyDefinition successor)
   {
      if (this.successor != null)
         this.successor.setParentJoint(null);

      this.successor = successor;

      if (this.successor != null)
         this.successor.setParentJoint(this);
   }

   public RigidBodyDefinition getSuccessor()
   {
      return successor;
   }

   public JointDefinition getParentJoint()
   {
      if (predecessor == null)
         return null;
      else
         return predecessor.getParentJoint();
   }

   public abstract void setInitialJointState(JointStateReadOnly initialJointState);

   public abstract JointStateBasics getInitialJointState();

   @XmlElement(name = "sensor")
   public void setSensorDefinitions(List<SensorDefinition> sensorDefinitions)
   {
      this.sensorDefinitions = sensorDefinitions;
   }

   public void addSensorDefinition(SensorDefinition sensorDefinition)
   {
      sensorDefinitions.add(sensorDefinition);
   }

   public List<SensorDefinition> getSensorDefinitions()
   {
      return sensorDefinitions;
   }

   public <T extends SensorDefinition> List<T> getSensorDefinitions(Class<T> sensorType)
   {
      return sensorDefinitions.stream().filter(sensorType::isInstance).map(sensorType::cast).collect(Collectors.toList());
   }

   @XmlElement(name = "kinematicPoint")
   public void setKinematicPointDefinitions(List<KinematicPointDefinition> kinematicPointDefinitions)
   {
      this.kinematicPointDefinitions = kinematicPointDefinitions;
   }

   public void addKinematicPointDefinition(KinematicPointDefinition kinematicPointDefinition)
   {
      kinematicPointDefinitions.add(kinematicPointDefinition);
   }

   public List<KinematicPointDefinition> getKinematicPointDefinitions()
   {
      return kinematicPointDefinitions;
   }

   @XmlElement(name = "externalWrenchPoint")
   public void setExternalWrenchPointDefinitions(List<ExternalWrenchPointDefinition> externalWrenchPointDefinitions)
   {
      this.externalWrenchPointDefinitions = externalWrenchPointDefinitions;
   }

   public void addExternalWrenchPointDefinition(ExternalWrenchPointDefinition externalWrenchPointDefinition)
   {
      externalWrenchPointDefinitions.add(externalWrenchPointDefinition);
   }

   public List<ExternalWrenchPointDefinition> getExternalWrenchPointDefinitions()
   {
      return externalWrenchPointDefinitions;
   }

   @XmlElement(name = "groundContactPoint")
   public void setGroundContactPointDefinitions(List<GroundContactPointDefinition> groundContactPointDefinitions)
   {
      this.groundContactPointDefinitions = groundContactPointDefinitions;
   }

   public void addGroundContactPointDefinition(GroundContactPointDefinition groundContactPointDefinition)
   {
      groundContactPointDefinitions.add(groundContactPointDefinition);
   }

   public List<GroundContactPointDefinition> getGroundContactPointDefinitions()
   {
      return groundContactPointDefinitions;
   }

   public void setLoopClosureDefinition(LoopClosureDefinition loopClosureDefinition)
   {
      this.loopClosureDefinition = loopClosureDefinition;
   }

   public boolean isLoopClosure()
   {
      return loopClosureDefinition != null;
   }

   public LoopClosureDefinition getLoopClosureDefinition()
   {
      return loopClosureDefinition;
   }

   public abstract JointBasics toJoint(RigidBodyBasics predecessor);

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(transformToParent);
      kinematicPointDefinitions.forEach(kp -> kp.applyTransform(transform));
      externalWrenchPointDefinitions.forEach(efp -> efp.applyTransform(transform));
      groundContactPointDefinitions.forEach(gcp -> gcp.applyTransform(transform));
      sensorDefinitions.forEach(sensor -> sensor.applyTransform(transform));
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(transformToParent);
      kinematicPointDefinitions.forEach(kp -> kp.applyInverseTransform(transform));
      externalWrenchPointDefinitions.forEach(efp -> efp.applyInverseTransform(transform));
      groundContactPointDefinitions.forEach(gcp -> gcp.applyInverseTransform(transform));
      sensorDefinitions.forEach(sensor -> sensor.applyInverseTransform(transform));
   }

   public abstract JointDefinition copy();

   public JointDefinition copyRecursive()
   {
      JointDefinition copy = copy();
      if (!isLoopClosure()) // Prevent infinite copying loop, but needs to be addressed manually
         copy.setSuccessor(successor.copyRecursive());
      return copy;
   }

   @Override
   public String toString()
   {
      String successorString = successor == null ? "null" : successor.getName();
      return name + ": origin: " + transformToParent + ", successor: " + successorString;
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, name);
      bits = EuclidHashCodeTools.addToHashCode(bits, transformToParent);
      // The predecessor is not used. This joint's hash code depends on the subtree it is the root of.
      bits = EuclidHashCodeTools.addToHashCode(bits, successor);
      bits = EuclidHashCodeTools.addToHashCode(bits, sensorDefinitions);
      bits = EuclidHashCodeTools.addToHashCode(bits, kinematicPointDefinitions);
      bits = EuclidHashCodeTools.addToHashCode(bits, externalWrenchPointDefinitions);
      bits = EuclidHashCodeTools.addToHashCode(bits, groundContactPointDefinitions);
      bits = EuclidHashCodeTools.addToHashCode(bits, loopClosureDefinition);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      JointDefinition other = (JointDefinition) object;

      if (!Objects.equals(name, other.name))
         return false;
      if (!Objects.equals(transformToParent, other.transformToParent))
         return false;
      // The predecessors are not compared. We only compared from this joint to downstream towards the end-effectors.
      if (!Objects.equals(successor, other.successor))
         return false;

      if (!Objects.equals(sensorDefinitions, other.sensorDefinitions))
         return false;
      if (!Objects.equals(kinematicPointDefinitions, other.kinematicPointDefinitions))
         return false;
      if (!Objects.equals(externalWrenchPointDefinitions, other.externalWrenchPointDefinitions))
         return false;
      if (!Objects.equals(groundContactPointDefinitions, other.groundContactPointDefinitions))
         return false;

      if (!Objects.equals(loopClosureDefinition, other.loopClosureDefinition))
         return false;

      return true;
   }
}
