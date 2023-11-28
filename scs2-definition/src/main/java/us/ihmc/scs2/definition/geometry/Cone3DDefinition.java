package us.ihmc.scs2.definition.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

import javax.xml.bind.annotation.XmlElement;

/**
 * Definition for creating a 3D cone.
 * <p>
 * It is assumed that the cone's axis is aligned with the z-axis and is positioned such that the
 * center of its bottom face is at the origin.
 * </p>
 */
public class Cone3DDefinition extends GeometryDefinition
{
   private double height;
   private double radius;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Cone3DDefinition()
   {
      setName("cone");
   }

   /**
    * Creates and initializes a definition for a regular 3D cone.
    *
    * @param height the height of the cone.
    * @param radius the radius of the base.
    */
   public Cone3DDefinition(double height, double radius)
   {
      this();
      this.height = height;
      this.radius = radius;
   }

   /**
    * Creates and initializes a definition for a regular 3D cone.
    *
    * @param height     the height of the cone.
    * @param radius     the radius of the base.
    * @param resolution used for discretizing the geometry.
    */
   public Cone3DDefinition(double height, double radius, int resolution)
   {
      this();
      this.height = height;
      this.radius = radius;
      this.resolution = resolution;
   }

   public Cone3DDefinition(Cone3DDefinition other)
   {
      setName(other.getName());
      height = other.height;
      radius = other.radius;
      resolution = other.resolution;
   }

   /**
    * Sets the height of the cone.
    *
    * @param height the height of the cone.
    */
   @XmlElement
   public void setHeight(double height)
   {
      this.height = height;
   }

   /**
    * Sets the radius of the cone.
    *
    * @param radius the radius of the base.
    */
   @XmlElement
   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   /**
    * Sets the cone's resolution used when discretizing it.
    *
    * @param resolution the cone's resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the height of the cone.
    *
    * @return the cone's height.
    */
   public double getHeight()
   {
      return height;
   }

   /**
    * Returns the radius of the cone.
    *
    * @return the radius of the base.
    */
   public double getRadius()
   {
      return radius;
   }

   /**
    * Returns the cone's resolution which can be used for discretizing it.
    *
    * @return the cone's resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public Cone3DDefinition copy()
   {
      return new Cone3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, height);
      bits = EuclidHashCodeTools.addToHashCode(bits, radius);
      bits = EuclidHashCodeTools.addToHashCode(bits, resolution);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      Cone3DDefinition other = (Cone3DDefinition) object;

      if (!EuclidCoreTools.equals(height, other.height))
         return false;
      if (!EuclidCoreTools.equals(radius, other.radius))
         return false;
      if (resolution != other.resolution)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "Cone: [name: " + getName() + ", height: " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, height) + ", radius: " + String.format(
            EuclidCoreIOTools.DEFAULT_FORMAT,
            radius) + ", resolution: " + resolution + "]";
   }
}
