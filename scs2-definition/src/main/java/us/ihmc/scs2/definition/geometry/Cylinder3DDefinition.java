package us.ihmc.scs2.definition.geometry;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * Definition for creating a 3D cylinder.
 * <p>
 * It is assumed that the cylinder's axis is aligned with the z-axis and it is centered at the
 * origin.
 * </p>
 */
public class Cylinder3DDefinition extends GeometryDefinition
{
   private double length;
   private double radius;
   private boolean centered = true;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Cylinder3DDefinition()
   {
      setName("cylinder");
   }

   /**
    * Creates and initializes a definition for a 3D cylinder.
    * <p>
    * The cylinder is assumed to be centered at the origin.
    * </p>
    * 
    * @param radius the cylinder's radius.
    * @param length the cylinder's length or height.
    */
   public Cylinder3DDefinition(double length, double radius)
   {
      this();
      this.length = length;
      this.radius = radius;
   }

   /**
    * Creates and initializes a definition for a 3D cylinder.
    * <p>
    * The cylinder is assumed to be centered at the origin.
    * </p>
    * 
    * @param radius     the cylinder's radius.
    * @param length     the cylinder's length or height.
    * @param resolution used for discretizing the geometry.
    */
   public Cylinder3DDefinition(double length, double radius, int resolution)
   {
      this();
      this.length = length;
      this.radius = radius;
      this.resolution = resolution;
   }

   /**
    * Creates and initializes a definition for a 3D cylinder.
    * 
    * @param radius   the cylinder's radius.
    * @param length   the cylinder's length or height.
    * @param centered {@code true} to center the cylinder are the origin, {@code false} to center its
    *                 bottom face at the origin.
    */
   public Cylinder3DDefinition(double length, double radius, boolean centered)
   {
      this();
      this.length = length;
      this.radius = radius;
      this.centered = centered;
   }

   /**
    * Creates and initializes a definition for a 3D cylinder.
    * 
    * @param radius     the cylinder's radius.
    * @param length     the cylinder's length or height.
    * @param centered   {@code true} to center the cylinder are the origin, {@code false} to center its
    *                   bottom face at the origin.
    * @param resolution used for discretizing the geometry.
    */
   public Cylinder3DDefinition(double length, double radius, boolean centered, int resolution)
   {
      this();
      this.length = length;
      this.radius = radius;
      this.centered = centered;
      this.resolution = resolution;
   }

   public Cylinder3DDefinition(Cylinder3DDefinition other)
   {
      setName(other.getName());
      length = other.length;
      radius = other.radius;
      centered = other.centered;
      resolution = other.resolution;
   }

   /**
    * Sets the length of the cylinder.
    * 
    * @param length the cylinder's length or height.
    */
   @XmlElement
   public void setLength(double length)
   {
      this.length = length;
   }

   /**
    * Sets the radius of the cylinder.
    * 
    * @param radius the cylinder's radius.
    */
   @XmlElement
   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   /**
    * Sets whether the cylinder should be centered at the origin or if its bottom face should be.
    * 
    * @param centered {@code true} for the cylinder to be centered at the origin, {@code false} for the
    *                 bottom face to be centered at the origin.
    */
   @XmlElement
   public void setCentered(boolean centered)
   {
      this.centered = centered;
   }

   /**
    * Sets the cylinder's resolution used when discretizing it.
    * 
    * @param resolution the cylinder's resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the length of the cylinder.
    * 
    * @return the cylinder's length or height.
    */
   public double getLength()
   {
      return length;
   }

   /**
    * Returns the radius of the cylinder.
    * 
    * @return the cylinder's radius.
    */
   public double getRadius()
   {
      return radius;
   }

   /**
    * Returns whether the cylinder should be centered at the origin.
    * 
    * @return {@code true} if the cylinder should be centered at the origin, {@code false} if its
    *         bottom face should centered at the origin.
    */
   public boolean isCentered()
   {
      return centered;
   }

   /**
    * Returns the cylinder's resolution which can be used for discretizing it.
    * 
    * @return the cylinder's resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public Cylinder3DDefinition copy()
   {
      return new Cylinder3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, length);
      bits = EuclidHashCodeTools.addToHashCode(bits, radius);
      bits = EuclidHashCodeTools.addToHashCode(bits, centered);
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

      Cylinder3DDefinition other = (Cylinder3DDefinition) object;

      if (Double.doubleToLongBits(length) != Double.doubleToLongBits(other.length))
         return false;
      if (Double.doubleToLongBits(radius) != Double.doubleToLongBits(other.radius))
         return false;
      if (centered != other.centered)
         return false;
      if (resolution != other.resolution)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "Cylinder: [name: " + getName() + ", length: " + length + ", radius: " + radius + ", resolution: " + resolution + "]";
   }
}
