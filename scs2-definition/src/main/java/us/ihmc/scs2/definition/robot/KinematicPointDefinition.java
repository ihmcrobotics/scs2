package us.ihmc.scs2.definition.robot;

import us.ihmc.euclid.interfaces.Transformable;
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
      final int prime = 31;
      int result = 1;
      result = prime * result + ((name == null) ? 0 : name.hashCode());
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
      KinematicPointDefinition other = (KinematicPointDefinition) obj;
      if (name == null)
      {
         if (other.name != null)
            return false;
      }
      else if (!name.equals(other.name))
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
