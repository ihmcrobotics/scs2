package us.ihmc.scs2.definition.geometry;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Definition for creating a 3D box.
 */
public class Box3DDefinition extends GeometryDefinition
{
   private double sizeX;
   private double sizeY;
   private double sizeZ;
   private boolean centered = true;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public Box3DDefinition()
   {
      setName("box");
   }

   /**
    * Creates and initializes a definition for a 3D box.
    * <p>
    * The box is centered at the origin.
    * </p>
    * 
    * @param sizeX box size along the x-axis.
    * @param sizeY box size along the y-axis.
    * @param sizeZ box size along the z-axis.
    */
   public Box3DDefinition(double sizeX, double sizeY, double sizeZ)
   {
      this(sizeX, sizeY, sizeZ, true);
   }

   /**
    * Creates and initializes a definition for a 3D box.
    * <p>
    * The box is centered at the origin.
    * </p>
    * 
    * @param sizeX    box size along the x-axis.
    * @param sizeY    box size along the y-axis.
    * @param sizeZ    box size along the z-axis.
    * @param centered when {@code true} the box centered at the origin, when {@code false} the bottom
    *                 face (the face at min z-coordinate) is centered at the origin.
    */
   public Box3DDefinition(double sizeX, double sizeY, double sizeZ, boolean centered)
   {
      this();
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;
      this.centered = centered;
   }

   /**
    * Creates and initializes a definition for a 3D box.
    * <p>
    * The box is centered at the origin.
    * </p>
    * 
    * @param size box size along each axis.
    */
   public Box3DDefinition(Tuple3DReadOnly size)
   {
      this(size.getX(), size.getY(), size.getZ());
   }

   public Box3DDefinition(Box3DDefinition other)
   {
      setName(other.getName());
      sizeX = other.sizeX;
      sizeY = other.sizeY;
      sizeZ = other.sizeZ;
      centered = other.centered;
   }

   /**
    * Sets the size of the box along the x-axis.
    * 
    * @param sizeX the box size along the x-axis.
    */
   @XmlElement
   public void setSizeX(double sizeX)
   {
      this.sizeX = sizeX;
   }

   /**
    * Sets the size of the box along the y-axis.
    * 
    * @param sizeY the box size along the y-axis.
    */
   @XmlElement
   public void setSizeY(double sizeY)
   {
      this.sizeY = sizeY;
   }

   /**
    * Sets the size of the box along the z-axis.
    * 
    * @param sizeZ the box size along the z-axis.
    */
   @XmlElement
   public void setSizeZ(double sizeZ)
   {
      this.sizeZ = sizeZ;
   }

   /**
    * Sets the size of the box.
    * 
    * @param sizeX the box size along the x-axis.
    * @param sizeY the box size along the y-axis.
    * @param sizeZ the box size along the z-axis.
    */
   public void setSize(double sizeX, double sizeY, double sizeZ)
   {
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.sizeZ = sizeZ;
   }

   /**
    * Sets the size of the box.
    * 
    * @param size the size of the box. Not modified.
    */
   public void setSize(Tuple3DReadOnly size)
   {
      setSize(size.getX(), size.getY(), size.getZ());
   }

   /**
    * Sets whether the box should be centered at the origin or if its bottom face should be.
    * 
    * @param centered {@code true} for the box to be centered at the origin, {@code false} for the
    *                 bottom face to be centered at the origin.
    */
   @XmlElement
   public void setCentered(boolean centered)
   {
      this.centered = centered;
   }

   /**
    * Returns the size of the box along the x-axis.
    * 
    * @return the box size along the x-axis.
    */
   public double getSizeX()
   {
      return sizeX;
   }

   /**
    * Returns the size of the box along the y-axis.
    * 
    * @return the box size along the y-axis.
    */
   public double getSizeY()
   {
      return sizeY;
   }

   /**
    * Returns the size of the box along the z-axis.
    * 
    * @return the box size along the z-axis.
    */
   public double getSizeZ()
   {
      return sizeZ;
   }

   /**
    * Returns whether the box should be centered at the origin.
    * 
    * @return {@code true} if the box should be centered at the origin, {@code false} if its bottom
    *         face should centered at the origin.
    */
   public boolean isCentered()
   {
      return centered;
   }

   @Override
   public Box3DDefinition copy()
   {
      return new Box3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, sizeX);
      bits = EuclidHashCodeTools.addToHashCode(bits, sizeY);
      bits = EuclidHashCodeTools.addToHashCode(bits, sizeZ);
      bits = EuclidHashCodeTools.addToHashCode(bits, centered);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (!super.equals(object))
         return false;

      Box3DDefinition other = (Box3DDefinition) object;
      if (Double.doubleToLongBits(sizeX) != Double.doubleToLongBits(other.sizeX))
         return false;
      if (Double.doubleToLongBits(sizeY) != Double.doubleToLongBits(other.sizeY))
         return false;
      if (Double.doubleToLongBits(sizeZ) != Double.doubleToLongBits(other.sizeZ))
         return false;
      if (centered != other.centered)
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getStringOf("Box: [name: " + getName() + ", size: (", ")]", ", ", sizeX, sizeY, sizeZ);
   }
}
