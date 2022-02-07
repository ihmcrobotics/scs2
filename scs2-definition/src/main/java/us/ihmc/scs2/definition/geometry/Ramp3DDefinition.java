package us.ihmc.scs2.definition.geometry;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Definition for creating a 3D ramp.
 * <p>
 * It is assumed that the ramp's bottom face is centered at the origin.
 * </p>
 */
public class Ramp3DDefinition extends GeometryDefinition
{
   private double sizeX;
   private double sizeY;
   private double sizeZ;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Ramp3DDefinition()
   {
      setName("ramp");
   }

   /**
    * Creates and initializes a definition for a 3D ramp.
    * 
    * @param sizeX the length of the ramp along the x-axis.
    * @param sizeY the width of the ramp along the y-axis.
    * @param sizeZ the height of the ramp along the z-axis.
    */
   public Ramp3DDefinition(double sizeX, double sizeY, double sizeZ)
   {
      this();
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;
   }

   /**
    * Creates and initializes a definition for a 3D ramp.
    * 
    * @param size the size of the ramp along the 3 axes.
    */
   public Ramp3DDefinition(Tuple3DReadOnly size)
   {
      this(size.getX(), size.getY(), size.getZ());
   }

   public Ramp3DDefinition(Ramp3DDefinition other)
   {
      setName(other.getName());
      sizeX = other.sizeX;
      sizeY = other.sizeY;
      sizeZ = other.sizeZ;
   }

   /**
    * Sets the length of the ramp along the x-axis.
    * 
    * @param sizeX the length of the ramp along the x-axis.
    */
   @XmlElement
   public void setSizeX(double sizeX)
   {
      this.sizeX = sizeX;
   }

   /**
    * Sets the width of the ramp along the y-axis.
    * 
    * @param sizeY the width of the ramp along the y-axis.
    */
   @XmlElement
   public void setSizeY(double sizeY)
   {
      this.sizeY = sizeY;
   }

   /**
    * Sets the height of the ramp along the z-axis.
    * 
    * @param sizeZ the height of the ramp along the z-axis.
    */
   @XmlElement
   public void setSizeZ(double sizeZ)
   {
      this.sizeZ = sizeZ;
   }

   /**
    * Sets the size of the ramp.
    * 
    * @param sizeX the length of the ramp along the x-axis.
    * @param sizeY the width of the ramp along the y-axis.
    * @param sizeZ the height of the ramp along the z-axis.
    */
   public void setSize(double sizeX, double sizeY, double sizeZ)
   {
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;
   }

   /**
    * Returns the length of the ramp along the x-axis.
    * 
    * @return the length of the ramp along the x-axis.
    */
   public double getSizeX()
   {
      return sizeX;
   }

   /**
    * Returns the width of the ramp along the y-axis.
    * 
    * @return the width of the ramp along the y-axis.
    */
   public double getSizeY()
   {
      return sizeY;
   }

   /**
    * Returns the height of the ramp along the z-axis.
    * 
    * @return the height of the ramp along the z-axis.
    */
   public double getSizeZ()
   {
      return sizeZ;
   }

   @Override
   public Ramp3DDefinition copy()
   {
      return new Ramp3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, sizeX);
      bits = EuclidHashCodeTools.addToHashCode(bits, sizeY);
      bits = EuclidHashCodeTools.addToHashCode(bits, sizeZ);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      Ramp3DDefinition other = (Ramp3DDefinition) object;

      if (Double.doubleToLongBits(sizeX) != Double.doubleToLongBits(other.sizeX))
         return false;
      if (Double.doubleToLongBits(sizeY) != Double.doubleToLongBits(other.sizeY))
         return false;
      if (Double.doubleToLongBits(sizeZ) != Double.doubleToLongBits(other.sizeZ))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getStringOf("Ramp: [name: " + getName() + ", size: (", ")]", ", ", sizeX, sizeY, sizeZ);
   }
}
