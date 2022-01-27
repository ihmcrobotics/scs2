package us.ihmc.scs2.definition.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlTransient;

import us.ihmc.euclid.interfaces.Transformable;
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
      final int prime = 31;
      int result = 1;
      result = prime * result + ((externalWrenchPointDefinitions == null) ? 0 : externalWrenchPointDefinitions.hashCode());
      result = prime * result + ((groundContactPointDefinitions == null) ? 0 : groundContactPointDefinitions.hashCode());
      result = prime * result + ((kinematicPointDefinitions == null) ? 0 : kinematicPointDefinitions.hashCode());
      result = prime * result + ((loopClosureDefinition == null) ? 0 : loopClosureDefinition.hashCode());
      result = prime * result + ((name == null) ? 0 : name.hashCode());
      result = prime * result + ((sensorDefinitions == null) ? 0 : sensorDefinitions.hashCode());
      result = prime * result + ((successor == null) ? 0 : successor.hashCode());
      result = prime * result + ((transformToParent == null) ? 0 : transformToParent.hashCode());
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
      JointDefinition other = (JointDefinition) obj;
      if (externalWrenchPointDefinitions == null)
      {
         if (other.externalWrenchPointDefinitions != null)
            return false;
      }
      else if (!externalWrenchPointDefinitions.equals(other.externalWrenchPointDefinitions))
         return false;
      if (groundContactPointDefinitions == null)
      {
         if (other.groundContactPointDefinitions != null)
            return false;
      }
      else if (!groundContactPointDefinitions.equals(other.groundContactPointDefinitions))
         return false;
      if (kinematicPointDefinitions == null)
      {
         if (other.kinematicPointDefinitions != null)
            return false;
      }
      else if (!kinematicPointDefinitions.equals(other.kinematicPointDefinitions))
         return false;
      if (loopClosureDefinition == null)
      {
         if (other.loopClosureDefinition != null)
            return false;
      }
      else if (!loopClosureDefinition.equals(other.loopClosureDefinition))
         return false;
      if (name == null)
      {
         if (other.name != null)
            return false;
      }
      else if (!name.equals(other.name))
         return false;
      if (predecessor == null)
      {
         if (other.predecessor != null)
            return false;
      }
      else if (!predecessor.equals(other.predecessor))
         return false;
      if (sensorDefinitions == null)
      {
         if (other.sensorDefinitions != null)
            return false;
      }
      else if (!sensorDefinitions.equals(other.sensorDefinitions))
         return false;
      if (successor == null)
      {
         if (other.successor != null)
            return false;
      }
      else if (!successor.equals(other.successor))
         return false;
      if (transformToParent == null)
      {
         if (other.transformToParent != null)
            return false;
      }
      else if (!transformToParent.equals(other.transformToParent))
         return false;
      return true;
   }
}
