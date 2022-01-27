package us.ihmc.scs2.definition.collision;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;

public class CollisionShapeDefinition
{
   private String name;
   private YawPitchRollTransformDefinition originPose = new YawPitchRollTransformDefinition();
   private GeometryDefinition geometryDefinition;

   /**
    * Collision identifier for this collidable. Use {@link CollidableHelper} to compute collision masks
    * and groups.
    */
   private long collisionMask = -1L;
   /**
    * Collision identifiers of other collidables that this collidable is allowed to collide with. Use
    * {@link CollidableHelper} to compute collision masks and groups.
    */
   private long collisionGroup = -1L;

   public CollisionShapeDefinition()
   {
   }

   public CollisionShapeDefinition(GeometryDefinition geometryDefinition)
   {
      this.geometryDefinition = geometryDefinition;
   }

   public CollisionShapeDefinition(RigidBodyTransformReadOnly originPose, GeometryDefinition geometryDefinition)
   {
      this(new YawPitchRollTransformDefinition(originPose), geometryDefinition);
   }

   public CollisionShapeDefinition(YawPitchRollTransformDefinition originPose, GeometryDefinition geometryDefinition)
   {
      this.originPose = originPose;
      this.geometryDefinition = geometryDefinition;
   }

   public CollisionShapeDefinition(CollisionShapeDefinition other)
   {
      name = other.name;
      originPose.set(other.originPose);
      if (other.geometryDefinition != null)
         geometryDefinition = other.geometryDefinition.copy();
   }

   @XmlElement
   public void setName(String name)
   {
      this.name = name;
   }

   public void setOriginPose(RigidBodyTransformReadOnly originPose)
   {
      this.originPose.set(originPose);
   }

   @XmlElement
   public void setOriginPose(YawPitchRollTransformDefinition originPose)
   {
      this.originPose = originPose;
   }

   @XmlElement
   public void setGeometryDefinition(GeometryDefinition geometryDefinition)
   {
      this.geometryDefinition = geometryDefinition;
   }

   @XmlElement
   public void setCollisionMask(long collisionMask)
   {
      this.collisionMask = collisionMask;
   }

   @XmlElement
   public void setCollisionGroup(long collisionGroup)
   {
      this.collisionGroup = collisionGroup;
   }

   public String getName()
   {
      return name;
   }

   public YawPitchRollTransformDefinition getOriginPose()
   {
      return originPose;
   }

   public GeometryDefinition getGeometryDefinition()
   {
      return geometryDefinition;
   }

   public long getCollisionMask()
   {
      return collisionMask;
   }

   public long getCollisionGroup()
   {
      return collisionGroup;
   }

   public CollisionShapeDefinition copy()
   {
      return new CollisionShapeDefinition(this);
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + (int) (collisionGroup ^ (collisionGroup >>> 32));
      result = prime * result + (int) (collisionMask ^ (collisionMask >>> 32));
      result = prime * result + ((geometryDefinition == null) ? 0 : geometryDefinition.hashCode());
      result = prime * result + ((name == null) ? 0 : name.hashCode());
      result = prime * result + ((originPose == null) ? 0 : originPose.hashCode());
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
      CollisionShapeDefinition other = (CollisionShapeDefinition) obj;
      if (collisionGroup != other.collisionGroup)
         return false;
      if (collisionMask != other.collisionMask)
         return false;
      if (geometryDefinition == null)
      {
         if (other.geometryDefinition != null)
            return false;
      }
      else if (!geometryDefinition.equals(other.geometryDefinition))
         return false;
      if (name == null)
      {
         if (other.name != null)
            return false;
      }
      else if (!name.equals(other.name))
         return false;
      if (originPose == null)
      {
         if (other.originPose != null)
            return false;
      }
      else if (!originPose.equals(other.originPose))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "CollisionShapeDefinition [name=" + name + ", originPose=" + originPose + ", geometryDefinition=" + geometryDefinition + ", collisionMask="
            + collisionMask + ", collisionGroup=" + collisionGroup + "]";
   }
}
