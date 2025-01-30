package us.ihmc.scs2.definition.visual;

import java.util.Objects;

import jakarta.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;

/**
 * A {@code VisualDefinition} represents a template for creating 3D graphics that can be used to
 * render a piece of the environment or a body of a robot.
 * <p>
 * See the helper class {@link VisualDefinitionFactory} that adds convenience methods for creating
 * new visuals.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class VisualDefinition
{
   /** A descriptive/representative name to be associated with this visual. */
   private String name;
   /**
    * The pose of the geometry.
    * <p>
    * When this visual is used for rendering a body of a robot, the pose is relative to the frame after
    * the parent joint of the visualized body.
    * </p>
    */
   private AffineTransformDefinition originPose = new AffineTransformDefinition();
   /**
    * The geometry of this visual. It can for instance be a primitive such as a
    * {@link Sphere3DDefinition} or a link to a model file {@link ModelFileGeometryDefinition}.
    */
   private GeometryDefinition geometryDefinition;
   /**
    * The material to apply on the geometry. It can be as simple as just a diffuse color using
    * {@link MaterialDefinition#MaterialDefinition(ColorDefinition)} or a texture
    * {@link MaterialDefinition#MaterialDefinition(TextureDefinition)}.
    */
   private MaterialDefinition materialDefinition;

   /**
    * Creates an empty visual which fields need to be initialized.
    */
   public VisualDefinition()
   {
   }

   /**
    * Creates a new visual using a "zero" pose.
    * 
    * @param geometryDefinition the geometry to use.
    * @param diffuseColor       the diffuse color for creating this visual's material.
    */
   public VisualDefinition(GeometryDefinition geometryDefinition, ColorDefinition diffuseColor)
   {
      this(geometryDefinition, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates a new visual using a "zero" pose.
    * 
    * @param geometryDefinition the geometry to use.
    * @param materialDefinition the material to use.
    */
   public VisualDefinition(GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this.geometryDefinition = geometryDefinition;
      this.materialDefinition = materialDefinition;
   }

   /**
    * Creates a new visual.
    * 
    * @param originPosition     the position of the geometry. If this is a visual for a robot body, the
    *                           position is with respect to the parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param diffuseColor       the diffuse color for creating this visual's material.
    */
   public VisualDefinition(Tuple3DReadOnly originPosition, GeometryDefinition geometryDefinition, ColorDefinition diffuseColor)
   {
      this(originPosition, geometryDefinition, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates a new visual.
    * 
    * @param originPosition     the position of the geometry. If this is a visual for a robot body, the
    *                           position is with respect to the parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param materialDefinition the material to use.
    */
   public VisualDefinition(Tuple3DReadOnly originPosition, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this(new AffineTransform(new Quaternion(), originPosition), geometryDefinition, materialDefinition);
   }

   /**
    * Creates a new visual.
    * 
    * @param originPose         the pose of the geometry. If this is a visual for a robot body, the
    *                           pose is with respect to the parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param diffuseColor       the diffuse color for creating this visual's material.
    */
   public VisualDefinition(RigidBodyTransformReadOnly originPose, GeometryDefinition geometryDefinition, ColorDefinition diffuseColor)
   {
      this(originPose, geometryDefinition, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates a new visual.
    * 
    * @param originPose         the pose of the geometry. If this is a visual for a robot body, the
    *                           pose is with respect to the parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param materialDefinition the material to use.
    */
   public VisualDefinition(RigidBodyTransformReadOnly originPose, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this(new AffineTransform(originPose), geometryDefinition, materialDefinition);
   }

   /**
    * Creates a new visual.
    * 
    * @param originPose         the transform (translation, rotation, scale) of the geometry. If this
    *                           is a visual for a robot body, the transform is with respect to the
    *                           parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param diffuseColor       the diffuse color for creating this visual's material.
    */
   public VisualDefinition(AffineTransformReadOnly originPose, GeometryDefinition geometryDefinition, ColorDefinition diffuseColor)
   {
      this(originPose, geometryDefinition, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates a new visual.
    * 
    * @param originPose         the transform (translation, rotation, scale) of the geometry. If this
    *                           is a visual for a robot body, the transform is with respect to the
    *                           parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param materialDefinition the material to use.
    */
   public VisualDefinition(AffineTransformReadOnly originPose, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this(new AffineTransformDefinition(originPose), geometryDefinition, materialDefinition);
   }

   /**
    * Creates a new visual.
    * 
    * @param originPose         the transform (translation, rotation, scale) of the geometry. If this
    *                           is a visual for a robot body, the transform is with respect to the
    *                           parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param diffuseColor       the diffuse color for creating this visual's material.
    */
   public VisualDefinition(AffineTransformDefinition originPose, GeometryDefinition geometryDefinition, ColorDefinition diffuseColor)
   {
      this(originPose, geometryDefinition, new MaterialDefinition(diffuseColor));
   }

   /**
    * Creates a new visual.
    * 
    * @param originPose         the transform (translation, rotation, scale) of the geometry. If this
    *                           is a visual for a robot body, the transform is with respect to the
    *                           parent joint frame.
    * @param geometryDefinition the geometry to use.
    * @param materialDefinition the material to use.
    */
   public VisualDefinition(AffineTransformDefinition originPose, GeometryDefinition geometryDefinition, MaterialDefinition materialDefinition)
   {
      this.originPose = originPose;
      this.geometryDefinition = geometryDefinition;
      this.materialDefinition = materialDefinition;
   }

   /**
    * Copy constructor. This constructor performs a deep copy.
    * 
    * @param other the other visual to make a copy of.
    */
   public VisualDefinition(VisualDefinition other)
   {
      name = other.name;
      originPose.set(other.originPose);
      if (other.geometryDefinition != null)
         geometryDefinition = other.geometryDefinition.copy();
      if (other.materialDefinition != null)
         materialDefinition = other.materialDefinition.copy();
   }

   /**
    * Sets descriptive/representative name to be associated with this visual.
    * 
    * @param name this visual's name.
    */
   @XmlElement
   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Sets the pose of the geometry.
    * <p>
    * When this visual is used for rendering a body of a robot, the pose is relative to the frame after
    * the parent joint of the visualized body.
    * </p>
    * 
    * @param originPose the pose for the geometry.
    */
   public void setOriginPose(RigidBodyTransformReadOnly originPose)
   {
      this.originPose = new AffineTransformDefinition(originPose);
   }

   /**
    * Sets the transform (translation, rotation, scale) of the geometry.
    * <p>
    * When this visual is used for rendering a body of a robot, the transform is relative to the frame
    * after the parent joint of the visualized body.
    * </p>
    * 
    * @param originPose the transform for the geometry.
    */
   public void setOriginPose(AffineTransformReadOnly originPose)
   {
      this.originPose = new AffineTransformDefinition(originPose);
   }

   /**
    * Sets the transform (translation, rotation, scale) of the geometry.
    * <p>
    * When this visual is used for rendering a body of a robot, the transform is relative to the frame
    * after the parent joint of the visualized body.
    * </p>
    * 
    * @param originPose the transform for the geometry.
    */
   @XmlElement
   public void setOriginPose(AffineTransformDefinition originPose)
   {
      this.originPose = originPose;
   }

   /**
    * Sets the geometry of this visual. It can for instance be a primitive such as a
    * {@link Sphere3DDefinition} or a link to a model file {@link ModelFileGeometryDefinition}.
    * 
    * @param geometryDefinition this visual's geometry.
    */
   @XmlElement
   public void setGeometryDefinition(GeometryDefinition geometryDefinition)
   {
      this.geometryDefinition = geometryDefinition;
   }

   /**
    * Sets the material to apply on the geometry. It can be as simple as just a diffuse color using
    * {@link MaterialDefinition#MaterialDefinition(ColorDefinition)} or a texture
    * {@link MaterialDefinition#MaterialDefinition(TextureDefinition)}.
    * 
    * @param materialDefinition the material to use for this visual.
    */
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
