package us.ihmc.scs2.definition.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

import jakarta.xml.bind.annotation.XmlElement;

/**
 * Definition for creating a 3D sphere.
 */
public class Sphere3DDefinition extends GeometryDefinition
{
   private double radius;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Sphere3DDefinition()
   {
      setName("sphere");
   }

   /**
    * Creates and initializes a definition for a 3D sphere.
    *
    * @param radius the radius of the sphere.
    */
   public Sphere3DDefinition(double radius)
   {
      this();
      this.radius = radius;
   }

   /**
    * Creates and initializes a definition for a 3D sphere.
    *
    * @param radius     the radius of the sphere.
    * @param resolution used for discretizing the geometry.
    */
   public Sphere3DDefinition(double radius, int resolution)
   {
      this();
      this.radius = radius;
      this.resolution = resolution;
   }

   public Sphere3DDefinition(Sphere3DDefinition other)
   {
      setName(other.getName());
      radius = other.radius;
      resolution = other.resolution;
   }

   /**
    * Sets the radius of the sphere.
    *
    * @param radius the radius of the sphere.
    */
   @XmlElement
   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   /**
    * Sets the sphere's resolution used when discretizing it.
    *
    * @param resolution the sphere's resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the radius of the sphere.
    *
    * @return the radius of the sphere.
    */
   public double getRadius()
   {
      return radius;
   }

   /**
    * Returns the sphere's resolution which can be used for discretizing it.
    *
    * @return the sphere's resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public Sphere3DDefinition copy()
   {
      return new Sphere3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
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

      Sphere3DDefinition other = (Sphere3DDefinition) object;

      if (!EuclidCoreTools.equals(radius, other.radius))
         return false;
      if (resolution != other.resolution)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "Sphere: [name: " + getName() + ", radius: " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, radius) + ", resolution: " + resolution + "]";
   }
}
