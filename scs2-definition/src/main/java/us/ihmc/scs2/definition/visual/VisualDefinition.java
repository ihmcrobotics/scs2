package us.ihmc.scs2.definition.visual;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;

public class VisualDefinition
{
   private String name;
   private AffineTransformDefinition originPose = new AffineTransformDefinition();
   private GeometryDefinition geometryDefinition;
   private MaterialDefinition materialDefinition;

   public VisualDefinition()
   {
   }

   public VisualDefinition(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this.geometryDefinition = geometryDefinition;
      this.materialDefinition = materialDefinition;
   }

   public VisualDefinition(Tuple3DReadOnly originPosition, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this(new AffineTransform(new Quaternion(), originPosition), geometryDefinition, materialDefinition);
   }

   public VisualDefinition(RigidBodyTransformReadOnly originPose, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this(new AffineTransform(originPose), geometryDefinition, materialDefinition);
   }

   public VisualDefinition(AffineTransformReadOnly originPose, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this(new AffineTransformDefinition(originPose), geometryDefinition, materialDefinition);
   }

   public VisualDefinition(AffineTransformDefinition originPose, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this.originPose = originPose;
      this.geometryDefinition = geometryDefinition;
      this.materialDefinition = materialDefinition;
   }

   public VisualDefinition(VisualDefinition other)
   {
      name = other.name;
      originPose.set(other.originPose);
      if (other.geometryDefinition != null)
         geometryDefinition = other.geometryDefinition.copy();
      if (other.materialDefinition != null)
         materialDefinition = other.materialDefinition.copy();
   }

   @XmlElement
   public void setName(String name)
   {
      this.name = name;
   }

   public void setOriginPose(RigidBodyTransformReadOnly originPose)
   {
      this.originPose = new AffineTransformDefinition(originPose);
   }

   public void setOriginPose(AffineTransformReadOnly originPose)
   {
      this.originPose = new AffineTransformDefinition(originPose);
   }

   @XmlElement
   public void setOriginPose(AffineTransformDefinition originPose)
   {
      this.originPose = originPose;
   }

   @XmlElement
   public void setGeometryDefinition(GeometryDefinition geometryDefinition)
   {
      this.geometryDefinition = geometryDefinition;
   }

   @XmlElement
   public void setMaterialDefinition(MaterialDefinition materialDefinition)
   {
      this.materialDefinition = materialDefinition;
   }

   public String getName()
   {
      return name;
   }

   public AffineTransformDefinition getOriginPose()
   {
      return originPose;
   }

   public GeometryDefinition getGeometryDefinition()
   {
      return geometryDefinition;
   }

   public MaterialDefinition getMaterialDefinition()
   {
      return materialDefinition;
   }

   public VisualDefinition copy()
   {
      return new VisualDefinition(this);
   }

   @Override
   public String toString()
   {
      return "VisualDefinition [name=" + name + ", originPose=" + originPose + ", geometryDefinition=" + geometryDefinition + ", materialDefinition="
            + materialDefinition + "]";
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, name);
      bits = EuclidHashCodeTools.addToHashCode(bits, originPose);
      bits = EuclidHashCodeTools.addToHashCode(bits, geometryDefinition);
      bits = EuclidHashCodeTools.addToHashCode(bits, materialDefinition);
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

      VisualDefinition other = (VisualDefinition) object;

      if (!Objects.equals(name, other.name))
         return false;
      if (!Objects.equals(originPose, other.originPose))
         return false;
      if (!Objects.equals(geometryDefinition, other.geometryDefinition))
         return false;
      if (!Objects.equals(materialDefinition, other.materialDefinition))
         return false;

      return true;
   }
}
