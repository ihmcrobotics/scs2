package us.ihmc.scs2.definition.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import jakarta.xml.bind.annotation.XmlElement;

/**
 * Definition for creating a 3D ellipsoid.
 */
public class Ellipsoid3DDefinition extends GeometryDefinition
{
   private double radiusX;
   private double radiusY;
   private double radiusZ;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Ellipsoid3DDefinition()
   {
      setName("ellipsoid");
   }

   /**
    * Creates and initializes a definition for a 3D ellipsoid.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    */
   public Ellipsoid3DDefinition(double radiusX, double radiusY, double radiusZ)
   {
      this();
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
   }

   /**
    * Creates and initializes a definition for a 3D ellipsoid.
    *
    * @param radiusX    radius of the ellipsoid along the x-axis.
    * @param radiusY    radius of the ellipsoid along the y-axis.
    * @param radiusZ    radius of the ellipsoid along the z-axis.
    * @param resolution used for discretizing the geometry.
    */
   public Ellipsoid3DDefinition(double radiusX, double radiusY, double radiusZ, int resolution)
   {
      this();
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
      this.resolution = resolution;
   }

   /**
    * Creates and initializes a definition for a 3D ellipsoid.
    *
    * @param radii the tuple containing the 3 radii of the ellipsoid.
    */
   public Ellipsoid3DDefinition(Tuple3DReadOnly radii)
   {
      this(radii.getX(), radii.getY(), radii.getZ());
   }

   /**
    * Creates and initializes a definition for a 3D ellipsoid.
    *
    * @param radii      the tuple containing the 3 radii of the ellipsoid.
    * @param resolution used for discretizing the geometry.
    */
   public Ellipsoid3DDefinition(Tuple3DReadOnly radii, int resolution)
   {
      this(radii.getX(), radii.getY(), radii.getZ(), resolution);
   }

   public Ellipsoid3DDefinition(Ellipsoid3DDefinition other)
   {
      setName(other.getName());
      radiusX = other.radiusX;
      radiusY = other.radiusY;
      radiusZ = other.radiusZ;
      resolution = other.resolution;
   }

   /**
    * Tests if this ellipsoid is a sphere, i.e. all three radii are equal.
    *
    * @return {@code true} if this ellipsoid is a sphere, {@code false} if it is an ellipsoid.
    */
   public boolean isSphere()
   {
      return radiusX == radiusY && radiusX == radiusZ;
   }

   /**
    * Sets the radius along the x-axis for the ellipsoid.
    *
    * @param radiusX the ellipsoid's radius along the x-axis.
    */
   @XmlElement
   public void setRadiusX(double radiusX)
   {
      this.radiusX = radiusX;
   }

   /**
    * Sets the radius along the y-axis for the ellipsoid.
    *
    * @param radiusY the ellipsoid's radius along the y-axis.
    */
   @XmlElement
   public void setRadiusY(double radiusY)
   {
      this.radiusY = radiusY;
   }

   /**
    * Sets the radius along the z-axis for the ellipsoid.
    *
    * @param radiusZ the ellipsoid's radius along the z-axis.
    */
   @XmlElement
   public void setRadiusZ(double radiusZ)
   {
      this.radiusZ = radiusZ;
   }

   /**
    * Sets the radii of the ellipsoid.
    *
    * @param radiusX the ellipsoid's radius along the x-axis.
    * @param radiusY the ellipsoid's radius along the y-axis.
    * @param radiusZ the ellipsoid's radius along the z-axis.
    */
   public void setRadii(double radiusX, double radiusY, double radiusZ)
   {
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
   }

   /**
    * Sets the ellipsoid's resolution used when discretizing it.
    *
    * @param resolution the ellipsoid's resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the radius along the x-axis of the ellipsoid.
    *
    * @return the ellipsoid's radius along the x-axis.
    */
   public double getRadiusX()
   {
      return radiusX;
   }

   /**
    * Returns the radius along the y-axis of the ellipsoid.
    *
    * @return the ellipsoid's radius along the y-axis.
    */
   public double getRadiusY()
   {
      return radiusY;
   }

   /**
    * Returns the radius along the z-axis of the ellipsoid.
    *
    * @return the ellipsoid's radius along the z-axis.
    */
   public double getRadiusZ()
   {
      return radiusZ;
   }

   /**
    * Returns the ellipsoid's resolution which can be used for discretizing it.
    *
    * @return the ellipsoid's resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public Ellipsoid3DDefinition copy()
   {
      return new Ellipsoid3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
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

      Ellipsoid3DDefinition other = (Ellipsoid3DDefinition) object;

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
      return EuclidCoreIOTools.getStringOf("Ellipsoid: [name: " + getName() + ", radii: (",
                                           "), resolution: " + resolution + "]",
                                           ", ",
                                           radiusX,
                                           radiusY,
                                           radiusZ);
   }
}
