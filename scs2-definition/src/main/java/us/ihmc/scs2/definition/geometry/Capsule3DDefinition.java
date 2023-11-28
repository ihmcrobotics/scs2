package us.ihmc.scs2.definition.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

import javax.xml.bind.annotation.XmlElement;

/**
 * Definition for creating a 3D capsule.
 * <p>
 * It is assumed that the capsule's axis is aligned with the z-axis and it is centered at the
 * origin.
 * </p>
 */
public class Capsule3DDefinition extends GeometryDefinition
{
   private double length;
   private double radiusX;
   private double radiusY;
   private double radiusZ;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Capsule3DDefinition()
   {
      setName("capsule");
   }

   /**
    * Creates and initializes a definition for a regular 3D capsule.
    *
    * @param length the capsule's length or height. Distance separating the center of the two half
    *               spheres.
    * @param radius the capsule's radius.
    */
   public Capsule3DDefinition(double length, double radius)
   {
      this(length, radius, radius, radius);
   }

   /**
    * Creates and initializes a definition for a 3D capsule.
    *
    * @param length  the capsule's length or height. Distance separating the center of the two half
    *                ellipsoids.
    * @param radiusX radius of along the x-axis.
    * @param radiusY radius of along the y-axis.
    * @param radiusZ radius of along the z-axis.
    */
   public Capsule3DDefinition(double length, double radiusX, double radiusY, double radiusZ)
   {
      this();
      this.length = length;
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
   }

   /**
    * Creates and initializes a definition for a regular 3D capsule.
    *
    * @param length     the capsule's length or height. Distance separating the center of the two half
    *                   spheres.
    * @param radius     the capsule's radius.
    * @param resolution used for discretizing the geometry.
    */
   public Capsule3DDefinition(double length, double radius, int resolution)
   {
      this(length, radius, radius, radius, resolution);
   }

   /**
    * Creates and initializes a definition for a 3D capsule.
    *
    * @param length     the capsule's length or height. Distance separating the center of the two half
    *                   ellipsoids.
    * @param radiusX    radius of the capsule along the x-axis.
    * @param radiusY    radius of the capsule along the y-axis.
    * @param radiusZ    radius of the capsule along the z-axis.
    * @param resolution used for discretizing the geometry.
    */
   public Capsule3DDefinition(double length, double radiusX, double radiusY, double radiusZ, int resolution)
   {
      this();
      this.length = length;
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
      this.resolution = resolution;
   }

   public Capsule3DDefinition(Capsule3DDefinition other)
   {
      setName(other.getName());
      length = other.length;
      radiusX = other.radiusX;
      radiusY = other.radiusY;
      radiusZ = other.radiusZ;
      resolution = other.resolution;
   }

   /**
    * Sets the length of the capsule.
    *
    * @param length the capsule's length or height. Distance separating the center of the two half
    *               ellipsoids.
    */
   @XmlElement
   public void setLength(double length)
   {
      this.length = length;
   }

   /**
    * Tests whether the capsule is a regular capsule, i.e. all radii are equal.
    *
    * @return {@code true} if the capsule is regular, {@code false} otherwise.
    */
   public boolean isRegular()
   {
      return radiusX == radiusY && radiusX == radiusZ;
   }

   /**
    * Sets the radius along the x-axis of the capsule.
    *
    * @param radiusX radius of the capsule along the x-axis.
    */
   @XmlElement
   public void setRadiusX(double radiusX)
   {
      this.radiusX = radiusX;
   }

   /**
    * Sets the radius along the y-axis of the capsule.
    *
    * @param radiusY radius of the capsule along the y-axis.
    */
   @XmlElement
   public void setRadiusY(double radiusY)
   {
      this.radiusY = radiusY;
   }

   /**
    * Sets the radius along the z-axis of the capsule.
    *
    * @param radiusZ radius of the capsule along the z-axis.
    */
   @XmlElement
   public void setRadiusZ(double radiusZ)
   {
      this.radiusZ = radiusZ;
   }

   /**
    * Sets the same radius along the three axes making the capsule regular.
    *
    * @param radius the capsule's radius.
    */
   public void setRadius(double radius)
   {
      setRadii(radius, radius, radius);
   }

   /**
    * Sets the radii for the capsule along each axis individually.
    *
    * @param radiusX radius of the capsule along the x-axis.
    * @param radiusY radius of the capsule along the y-axis.
    * @param radiusZ radius of the capsule along the z-axis.
    */
   public void setRadii(double radiusX, double radiusY, double radiusZ)
   {
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
   }

   /**
    * Sets the capsule's resolution used when discretizing it.
    *
    * @param resolution the capsule's resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the length of the capsule.
    *
    * @return the capsule's length or height. Distance separating the center of the two half
    *       ellipsoids.
    */
   public double getLength()
   {
      return length;
   }

   /**
    * Returns the capsule's radius along the x-axis.
    *
    * @return the radius along the x-axis.
    */
   public double getRadiusX()
   {
      return radiusX;
   }

   /**
    * Returns the capsule's radius along the y-axis.
    *
    * @return the radius along the y-axis.
    */
   public double getRadiusY()
   {
      return radiusY;
   }

   /**
    * Returns the capsule's radius along the z-axis.
    *
    * @return the radius along the z-axis.
    */
   public double getRadiusZ()
   {
      return radiusZ;
   }

   /**
    * Returns the capsule's resolution which can be used for discretizing it.
    *
    * @return the capsule's resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public Capsule3DDefinition copy()
   {
      return new Capsule3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, length);
      bits = EuclidHashCodeTools.addToHashCode(bits, radiusX);
      bits = EuclidHashCodeTools.addToHashCode(bits, radiusY);
      bits = EuclidHashCodeTools.addToHashCode(bits, radiusZ);
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

      Capsule3DDefinition other = (Capsule3DDefinition) object;

      if (!EuclidCoreTools.equals(length, other.length))
         return false;
      if (!EuclidCoreTools.equals(radiusX, other.radiusX))
         return false;
      if (!EuclidCoreTools.equals(radiusY, other.radiusY))
         return false;
      if (!EuclidCoreTools.equals(radiusZ, other.radiusZ))
         return false;
      if (resolution != other.resolution)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "Capsule: [name: " + getName() + ", length: " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, length) + EuclidCoreIOTools.getStringOf(
            ", radii: (",
            ")",
            ", ",
            radiusX,
            radiusY,
            radiusZ) + ", resolution: " + resolution + "]";
   }
}
