package us.ihmc.scs2.definition.geometry;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * Definition for creating a 3D torus.
 * <p>
 * It is assumed that the torus' axis is aligned with the z-axis and its centroid at the origin.
 * </p>
 */
public class Torus3DDefinition extends GeometryDefinition
{
   private double majorRadius;
   private double minorRadius;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Torus3DDefinition()
   {
      setName("torus");
   }

   /**
    * Creates and initializes a definition for a 3D torus.
    * 
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    */
   public Torus3DDefinition(double majorRadius, double minorRadius)
   {
      this();
      this.majorRadius = majorRadius;
      this.minorRadius = minorRadius;
   }

   /**
    * Creates and initializes a definition for a partial 3D torus.
    * 
    * @param majorRadius the radius from the torus centroid to the tube center.
    * @param minorRadius the radius of the tube.
    * @param resolution  used for discretizing the geometry.
    */
   public Torus3DDefinition(double majorRadius, double minorRadius, int resolution)
   {
      this();
      this.majorRadius = majorRadius;
      this.minorRadius = minorRadius;
      this.resolution = resolution;
   }

   public Torus3DDefinition(Torus3DDefinition other)
   {
      setName(other.getName());
      majorRadius = other.majorRadius;
      minorRadius = other.minorRadius;
      resolution = other.resolution;
   }

   /**
    * Sets the torus' major radius.
    * 
    * @param majorRadius the radius from the torus centroid to the tube center.
    */
   @XmlElement
   public void setMajorRadius(double majorRadius)
   {
      this.majorRadius = majorRadius;
   }

   /**
    * Sets the torus' minor radius.
    * 
    * @param minorRadius the radius of the tube.
    */
   @XmlElement
   public void setMinorRadius(double minorRadius)
   {
      this.minorRadius = minorRadius;
   }

   /**
    * Sets the torus' resolution used when discretizing it.
    * 
    * @param resolution the torus' resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the torus' major radius.
    * 
    * @return the radius from the torus centroid to the tube center.
    */
   public double getMajorRadius()
   {
      return majorRadius;
   }

   /**
    * Returns the torus' minor radius.
    * 
    * @return the radius of the tube.
    */
   public double getMinorRadius()
   {
      return minorRadius;
   }

   /**
    * Returns the torus' resolution which can be used for discretizing it.
    * 
    * @return the torus' resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public Torus3DDefinition copy()
   {
      return new Torus3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, majorRadius);
      bits = EuclidHashCodeTools.addToHashCode(bits, minorRadius);
      bits = EuclidHashCodeTools.addToHashCode(bits, resolution);
      bits = EuclidHashCodeTools.addToHashCode(bits, getName());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof Torus3DDefinition)
      {
         Torus3DDefinition other = (Torus3DDefinition) object;
         if (majorRadius != other.majorRadius)
            return false;
         if (minorRadius != other.minorRadius)
            return false;
         if (resolution != other.resolution)
            return false;
         return super.equals(object);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "Torus: [name: " + getName() + EuclidCoreIOTools.getStringOf(", radii: (", ")", ", ", majorRadius, minorRadius) + ", resolution: " + resolution
            + "]";
   }
}
