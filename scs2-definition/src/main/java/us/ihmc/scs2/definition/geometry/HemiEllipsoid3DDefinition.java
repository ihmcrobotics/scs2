package us.ihmc.scs2.definition.geometry;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

import jakarta.xml.bind.annotation.XmlElement;

/**
 * Definition for creating a 3D hemi-ellipsoid.
 * <p>
 * It is assumed that the hemi-ellipsoid represents the top half of a full ellipsoid, i.e. the half
 * in the z positive space.
 * </p>
 */
public class HemiEllipsoid3DDefinition extends GeometryDefinition
{
   private double radiusX;
   private double radiusY;
   private double radiusZ;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public HemiEllipsoid3DDefinition()
   {
      setName("hemiEllipsoid");
   }

   /**
    * Creates and initializes a definition for a 3D hemi-ellipsoid.
    *
    * @param radiusX radius of the hemi-ellipsoid along the x-axis.
    * @param radiusY radius of the hemi-ellipsoid along the y-axis.
    * @param radiusZ radius of the hemi-ellipsoid along the z-axis.
    */
   public HemiEllipsoid3DDefinition(double radiusX, double radiusY, double radiusZ)
   {
      this();
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
   }

   /**
    * Creates and initializes a definition for a 3D hemi-ellipsoid.
    *
    * @param radiusX    radius of the hemi-ellipsoid along the x-axis.
    * @param radiusY    radius of the hemi-ellipsoid along the y-axis.
    * @param radiusZ    radius of the hemi-ellipsoid along the z-axis.
    * @param resolution used for discretizing the geometry.
    */
   public HemiEllipsoid3DDefinition(double radiusX, double radiusY, double radiusZ, int resolution)
   {
      this();
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
      this.resolution = resolution;
   }

   public HemiEllipsoid3DDefinition(HemiEllipsoid3DDefinition other)
   {
      setName(other.getName());
      radiusX = other.radiusX;
      radiusY = other.radiusY;
      radiusZ = other.radiusZ;
      resolution = other.resolution;
   }

   /**
    * Sets the radius along the x-axis for the hemi-ellipsoid.
    *
    * @param radiusX the hemi-ellipsoid's radius along the x-axis.
    */
   @XmlElement
   public void setRadiusX(double radiusX)
   {
      this.radiusX = radiusX;
   }

   /**
    * Sets the radius along the y-axis for the hemi-ellipsoid.
    *
    * @param radiusY the hemi=ellipsoid's radius along the y-axis.
    */
   @XmlElement
   public void setRadiusY(double radiusY)
   {
      this.radiusY = radiusY;
   }

   /**
    * Sets the radius along the z-axis for the hemi-ellipsoid.
    *
    * @param radiusZ the hemi-ellipsoid's radius along the z-axis.
    */
   @XmlElement
   public void setRadiusZ(double radiusZ)
   {
      this.radiusZ = radiusZ;
   }

   /**
    * Sets the radii of the hemi-ellipsoid.
    *
    * @param radiusX the hemi-ellipsoid's radius along the x-axis.
    * @param radiusY the hemi-ellipsoid's radius along the y-axis.
    * @param radiusZ the hemi-ellipsoid's radius along the z-axis.
    */
   public void setRadii(double radiusX, double radiusY, double radiusZ)
   {
      this.radiusX = radiusX;
      this.radiusY = radiusY;
      this.radiusZ = radiusZ;
   }

   /**
    * Sets the hemi-ellipsoid's resolution used when discretizing it.
    *
    * @param resolution the hemi-ellipsoid's resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the radius along the x-axis of the hemi-ellipsoid.
    *
    * @return the hemi-ellipsoid's radius along the x-axis.
    */
   public double getRadiusX()
   {
      return radiusX;
   }

   /**
    * Returns the radius along the y-axis of the hemi-ellipsoid.
    *
    * @return the hemi-ellipsoid's radius along the y-axis.
    */
   public double getRadiusY()
   {
      return radiusY;
   }

   /**
    * Returns the radius along the z-axis of the hemi-ellipsoid.
    *
    * @return the hemi-ellipsoid's radius along the z-axis.
    */
   public double getRadiusZ()
   {
      return radiusZ;
   }

   /**
    * Returns the hemi-ellipsoid's resolution which can be used for discretizing it.
    *
    * @return the hemi-ellipsoid's resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public HemiEllipsoid3DDefinition copy()
   {
      return new HemiEllipsoid3DDefinition(this);
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

      HemiEllipsoid3DDefinition other = (HemiEllipsoid3DDefinition) object;

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
      return EuclidCoreIOTools.getStringOf("Hemi-ellipsoid: [name: " + getName() + ", radii: (",
                                           "), resolution: " + resolution + "]",
                                           ", ",
                                           radiusX,
                                           radiusY,
                                           radiusZ);
   }
}
