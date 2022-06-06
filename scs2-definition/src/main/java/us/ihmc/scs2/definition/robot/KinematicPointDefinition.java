package us.ihmc.scs2.definition.robot;

import java.util.Objects;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class KinematicPointDefinition implements Transformable
{
   private String name;
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();

   public KinematicPointDefinition()
   {
   }

   public KinematicPointDefinition(String name)
   {
      this.name = name;
   }

   public KinematicPointDefinition(String name, Tuple3DReadOnly offsetFromJoint)
   {
      this.name = name;
      this.transformToParent.getTranslation().set(offsetFromJoint);
   }

   public KinematicPointDefinition(String name, RigidBodyTransformReadOnly transformToParent)
   {
      this.name = name;
      this.transformToParent.set(transformToParent);
   }

   public KinematicPointDefinition(KinematicPointDefinition other)
   {
      name = other.name;
      transformToParent.set(other.transformToParent);
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   public void setTransformToParent(RigidBodyTransformReadOnly transformToParent)
   {
      this.transformToParent.set(transformToParent);
   }

   public KinematicPointDefinition copy()
   {
      return new KinematicPointDefinition(this);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(transformToParent);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(transformToParent);
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + " - " + name;
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, name);
      bits = EuclidHashCodeTools.addToHashCode(bits, transformToParent);
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

      KinematicPointDefinition other = (KinematicPointDefinition) object;

      if (!Objects.equals(name, other.name))
         return false;
      if (!Objects.equals(transformToParent, other.transformToParent))
         return false;

      return true;
   }
}
