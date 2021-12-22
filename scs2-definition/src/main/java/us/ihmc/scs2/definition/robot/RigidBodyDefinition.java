package us.ihmc.scs2.definition.robot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlTransient;
import javax.xml.bind.annotation.XmlType;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

@XmlType(propOrder = {"name", "mass", "momentOfInertia", "inertiaPose", "childrenJoints", "visualDefinitions", "collisionShapeDefinitions"})
public class RigidBodyDefinition implements Transformable
{
   private String name;
   private double mass;
   private MomentOfInertiaDefinition momentOfInertia = new MomentOfInertiaDefinition();
   private YawPitchRollTransformDefinition inertiaPose = new YawPitchRollTransformDefinition();

   private JointDefinition parentJoint;
   private List<JointDefinition> childrenJoints = new ArrayList<>();

   private List<VisualDefinition> visualDefinitions = new ArrayList<>();
   private List<CollisionShapeDefinition> collisionShapeDefinitions = new ArrayList<>();

   public RigidBodyDefinition()
   {
   }

   public RigidBodyDefinition(String name)
   {
      setName(name);
   }

   public RigidBodyDefinition(RigidBodyDefinition other)
   {
      name = other.name;
      mass = other.mass;
      momentOfInertia.set(other.momentOfInertia);
      inertiaPose.set(other.inertiaPose);
      for (VisualDefinition visualDefinition : other.visualDefinitions)
         visualDefinitions.add(visualDefinition.copy());
      for (CollisionShapeDefinition collisionShapeDefinition : other.collisionShapeDefinitions)
         collisionShapeDefinitions.add(collisionShapeDefinition.copy());
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
   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public double getMass()
   {
      return mass;
   }

   @XmlElement
   public void setMomentOfInertia(MomentOfInertiaDefinition momentOfInertia)
   {
      this.momentOfInertia = momentOfInertia;
   }

   public void setMomentOfInertia(Matrix3DReadOnly momentOfInertia)
   {
      this.momentOfInertia.set(momentOfInertia);
   }

   public MomentOfInertiaDefinition getMomentOfInertia()
   {
      return momentOfInertia;
   }

   @XmlElement
   public void setInertiaPose(YawPitchRollTransformDefinition inertiaPose)
   {
      this.inertiaPose = inertiaPose;
   }

   public void setInertiaPose(RigidBodyTransformReadOnly inertiaPose)
   {
      this.inertiaPose.set(inertiaPose);
   }

   public void setCenterOfMassOffset(double x, double y, double z)
   {
      inertiaPose.getTranslation().set(x, y, z);
   }

   public void setCenterOfMassOffset(Tuple3DReadOnly centerOfMassOffset)
   {
      setCenterOfMassOffset(centerOfMassOffset.getX(), centerOfMassOffset.getY(), centerOfMassOffset.getZ());
   }

   public YawPitchRollTransformDefinition getInertiaPose()
   {
      return inertiaPose;
   }

   public Vector3D getCenterOfMassOffset()
   {
      return inertiaPose.getTranslation();
   }

   @XmlTransient
   public void setParentJoint(JointDefinition parentJoint)
   {
      this.parentJoint = parentJoint;
   }

   public JointDefinition getParentJoint()
   {
      return parentJoint;
   }

   @XmlElement(name = "childJoint")
   public void setChildrenJoints(List<JointDefinition> childrenJoints)
   {
      if (this.childrenJoints != null)
      {
         for (JointDefinition childJoint : this.childrenJoints)
            childJoint.setPredecessor(null);
      }

      this.childrenJoints = childrenJoints;

      if (this.childrenJoints != null)
      {
         for (JointDefinition childJoint : this.childrenJoints)
            childJoint.setPredecessor(this);
      }
   }

   public void addChildJoint(JointDefinition childJoint)
   {
      childrenJoints.add(childJoint);
      childJoint.setPredecessor(this);
   }

   public void addChildJoints(Collection<? extends JointDefinition> childJoints)
   {
      childJoints.forEach(child -> addChildJoint(child));
   }

   public void removeChildJoint(JointDefinition childJoint)
   {
      if (childrenJoints.remove(childJoint))
         childJoint.setPredecessor(null);
   }

   public List<JointDefinition> getChildrenJoints()
   {
      return childrenJoints;
   }

   @XmlElement(name = "visual")
   public void setVisualDefinitions(List<VisualDefinition> visualDefinitions)
   {
      this.visualDefinitions = visualDefinitions;
   }

   public void addVisualDefinition(VisualDefinition visualDefinition)
   {
      visualDefinitions.add(visualDefinition);
   }

   public void addVisualDefinitions(Collection<VisualDefinition> visualDefinitions)
   {
      visualDefinitions.forEach(this::addVisualDefinition);
   }

   public List<VisualDefinition> getVisualDefinitions()
   {
      return visualDefinitions;
   }

   @XmlElement(name = "collision")
   public void setCollisionShapeDefinitions(List<CollisionShapeDefinition> collisionShapeDefinitions)
   {
      this.collisionShapeDefinitions = collisionShapeDefinitions;
   }

   public void addCollisionShapeDefinition(CollisionShapeDefinition collisionShapeDefinition)
   {
      collisionShapeDefinitions.add(collisionShapeDefinition);
   }

   public List<CollisionShapeDefinition> getCollisionShapeDefinitions()
   {
      return collisionShapeDefinitions;
   }

   public RigidBodyBasics toRootBody(ReferenceFrame parentStationaryFrame)
   {
      return new RigidBody(getName(), parentStationaryFrame);
   }

   public RigidBodyBasics toRigidBody(JointBasics parentJoint)
   {
      return new RigidBody(getName(), parentJoint, getMomentOfInertia(), getMass(), getInertiaPose());
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(inertiaPose);
      transform.transform(momentOfInertia);
      visualDefinitions.forEach(visual -> transform.transform(visual.getOriginPose()));
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(inertiaPose);
      transform.inverseTransform(momentOfInertia);
      visualDefinitions.forEach(visual -> transform.inverseTransform(visual.getOriginPose()));
   }

   public RigidBodyDefinition copy()
   {
      return new RigidBodyDefinition(this);
   }

   public RigidBodyDefinition copyRecursive()
   {
      RigidBodyDefinition copy = copy();
      for (JointDefinition childJoint : childrenJoints)
      {
         JointDefinition childJointCopy = childJoint.copyRecursive();
         copy.addChildJoint(childJointCopy);
      }
      return copy;
   }

   @Override
   public String toString()
   {
      String childrenString = childrenJoints == null ? "[]" : EuclidCoreIOTools.getCollectionString("[", "]", ", ", childrenJoints, JointDefinition::getName);
      return name + ": inertia pose: " + inertiaPose + ", children: " + childrenString;
   }
}
