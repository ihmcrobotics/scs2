package us.ihmc.scs2.definition.geometry;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * Definition for creating a truncated 3D cone.
 * <p>
 * It is assumed that the cone's axis is aligned with the z-axis and its centroid at the origin.
 * </p>
 */
public class TruncatedCone3DDefinition extends GeometryDefinition
{
   private double height;
   private double topRadiusX;
   private double topRadiusY;
   private double baseRadiusX;
   private double baseRadiusY;
   private boolean centered = false;
   private int resolution = 64;

   /**
    * Creates an empty definition. The parameters have to be all set before this definition can be
    * used.
    */
   public TruncatedCone3DDefinition()
   {
      setName("truncatedCone");
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone.
    * <p>
    * The truncated cone is assumed to have its base centered at the origin.
    * </p>
    *
    * @param height     the height of the truncated cone.
    * @param topRadius  the radius of the top face.
    * @param baseRadius the radius of the bottom face.
    */
   public TruncatedCone3DDefinition(double height, double topRadius, double baseRadius)
   {
      this(height, topRadius, topRadius, baseRadius, baseRadius);
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone.
    *
    * @param height     the height of the truncated cone.
    * @param topRadius  the radius of the top face.
    * @param baseRadius the radius of the bottom face.
    * @param centered   {@code true} to center the truncated cone are the origin, {@code false} to
    *                   center its bottom face at the origin.
    */
   public TruncatedCone3DDefinition(double height, double topRadius, double baseRadius, boolean centered)
   {
      this(height, topRadius, topRadius, baseRadius, baseRadius, centered);
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone.
    * <p>
    * The truncated cone is assumed to have its base centered at the origin.
    * </p>
    *
    * @param height     the height of the truncated cone.
    * @param topRadius  the radius of the top face.
    * @param baseRadius the radius of the bottom face.
    * @param resolution used for discretizing the geometry.
    */
   public TruncatedCone3DDefinition(double height, double topRadius, double baseRadius, int resolution)
   {
      this(height, topRadius, topRadius, baseRadius, baseRadius, resolution);
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone.
    *
    * @param height     the height of the truncated cone.
    * @param topRadius  the radius of the top face.
    * @param baseRadius the radius of the bottom face.
    * @param centered   {@code true} to center the truncated cone are the origin, {@code false} to
    *                   center its bottom face at the origin.
    * @param resolution used for discretizing the geometry.
    */
   public TruncatedCone3DDefinition(double height, double topRadius, double baseRadius, boolean centered, int resolution)
   {
      this(height, topRadius, topRadius, baseRadius, baseRadius, centered, resolution);
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone which top and bottom faces are
    * ellipses.
    * <p>
    * The truncated cone is assumed to have its base centered at the origin.
    * </p>
    *
    * @param height      the height of the truncated cone.
    * @param topRadiusX  the radius of the top face along the x-axis.
    * @param topRadiusY  the radius of the top face along the y-axis.
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    */
   public TruncatedCone3DDefinition(double height, double topRadiusX, double topRadiusY, double baseRadiusX, double baseRadiusY)
   {
      this();
      this.height = height;
      this.topRadiusX = topRadiusX;
      this.topRadiusY = topRadiusY;
      this.baseRadiusX = baseRadiusX;
      this.baseRadiusY = baseRadiusY;
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone which top and bottom faces are
    * ellipses.
    * <p>
    * The truncated cone is assumed to have its base centered at the origin.
    * </p>
    *
    * @param height      the height of the truncated cone.
    * @param topRadiusX  the radius of the top face along the x-axis.
    * @param topRadiusY  the radius of the top face along the y-axis.
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    * @param centered    {@code true} to center the truncated cone are the origin, {@code false} to
    *                    center its bottom face at the origin.
    */
   public TruncatedCone3DDefinition(double height, double topRadiusX, double topRadiusY, double baseRadiusX, double baseRadiusY, boolean centered)
   {
      this();
      this.height = height;
      this.topRadiusX = topRadiusX;
      this.topRadiusY = topRadiusY;
      this.baseRadiusX = baseRadiusX;
      this.baseRadiusY = baseRadiusY;
      this.centered = centered;
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone which top and bottom faces are
    * ellipses.
    * <p>
    * The truncated cone is assumed to have its base centered at the origin.
    * </p>
    *
    * @param height      the height of the truncated cone.
    * @param topRadiusX  the radius of the top face along the x-axis.
    * @param topRadiusY  the radius of the top face along the y-axis.
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    * @param resolution  used for discretizing the geometry.
    */
   public TruncatedCone3DDefinition(double height, double topRadiusX, double topRadiusY, double baseRadiusX, double baseRadiusY, int resolution)
   {
      this();
      this.height = height;
      this.topRadiusX = topRadiusX;
      this.topRadiusY = topRadiusY;
      this.baseRadiusX = baseRadiusX;
      this.baseRadiusY = baseRadiusY;
      this.resolution = resolution;
   }

   /**
    * Creates and initializes a definition for a truncated 3D cone which top and bottom faces are
    * ellipses.
    * <p>
    * The truncated cone is assumed to have its base centered at the origin.
    * </p>
    *
    * @param height      the height of the truncated cone.
    * @param topRadiusX  the radius of the top face along the x-axis.
    * @param topRadiusY  the radius of the top face along the y-axis.
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    * @param centered    {@code true} to center the truncated cone are the origin, {@code false} to
    *                    center its bottom face at the origin.
    * @param resolution  used for discretizing the geometry.
    */
   public TruncatedCone3DDefinition(double height,
                                    double topRadiusX,
                                    double topRadiusY,
                                    double baseRadiusX,
                                    double baseRadiusY,
                                    boolean centered,
                                    int resolution)
   {
      this();
      this.height = height;
      this.topRadiusX = topRadiusX;
      this.topRadiusY = topRadiusY;
      this.baseRadiusX = baseRadiusX;
      this.baseRadiusY = baseRadiusY;
      this.centered = centered;
      this.resolution = resolution;
   }

   public TruncatedCone3DDefinition(TruncatedCone3DDefinition other)
   {
      setName(other.getName());
      height = other.height;
      topRadiusX = other.topRadiusX;
      topRadiusY = other.topRadiusY;
      baseRadiusX = other.baseRadiusX;
      baseRadiusY = other.baseRadiusY;
      centered = other.centered;
      resolution = other.resolution;
   }

   /**
    * Sets the height of the truncated cone.
    *
    * @param height the height of the truncated cone.
    */
   @XmlElement
   public void setHeight(double height)
   {
      this.height = height;
   }

   /**
    * Sets the radius of the top face along the x-axis.
    *
    * @param topRadiusX the radius of the top face along the x-axis.
    */
   @XmlElement
   public void setTopRadiusX(double topRadiusX)
   {
      this.topRadiusX = topRadiusX;
   }

   /**
    * Sets the radius of the top face along the y-axis.
    *
    * @param topRadiusY the radius of the top face along the y-axis.
    */
   @XmlElement
   public void setTopRadiusY(double topRadiusY)
   {
      this.topRadiusY = topRadiusY;
   }

   /**
    * Sets the radius for the top face.
    *
    * @param topRadius the radius for the top face.
    */
   public void setTopRadius(double topRadius)
   {
      setTopRadii(topRadius, topRadius);
   }

   /**
    * Sets the radii for the top face.
    *
    * @param topRadiusX the radius of the top face along the x-axis.
    * @param topRadiusY the radius of the top face along the y-axis.
    */
   public void setTopRadii(double topRadiusX, double topRadiusY)
   {
      this.topRadiusX = topRadiusX;
      this.topRadiusY = topRadiusY;
   }

   /**
    * Sets the radius of the bottom along the x-axis.
    *
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    */
   @XmlElement
   public void setBaseRadiusX(double baseRadiusX)
   {
      this.baseRadiusX = baseRadiusX;
   }

   /**
    * Sets the radius of the bottom along the y-axis.
    *
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    */
   @XmlElement
   public void setBaseRadiusY(double baseRadiusY)
   {
      this.baseRadiusY = baseRadiusY;
   }

   /**
    * Sets the radius for the bottom face.
    *
    * @param baseRadius the radius for the bottom face.
    */
   public void setBaseRadius(double baseRadius)
   {
      setBaseRadii(baseRadius, baseRadius);
   }

   /**
    * Sets the radii for the bottom face.
    *
    * @param baseRadiusX the radius of the bottom face along the x-axis.
    * @param baseRadiusY the radius of the bottom face along the y-axis.
    */
   public void setBaseRadii(double baseRadiusX, double baseRadiusY)
   {
      this.baseRadiusX = baseRadiusX;
      this.baseRadiusY = baseRadiusY;
   }

   /**
    * Sets whether the truncated cone should be centered at the origin or if its bottom face should be.
    * 
    * @param centered {@code true} for the truncated cone to be centered at the origin, {@code false}
    *                 for the bottom face to be centered at the origin.
    */
   @XmlElement
   public void setCentered(boolean centered)
   {
      this.centered = centered;
   }

   /**
    * Sets the truncated cone's resolution used when discretizing it.
    * 
    * @param resolution the cone's resolution.
    */
   @XmlElement
   public void setResolution(int resolution)
   {
      this.resolution = resolution;
   }

   /**
    * Returns the height of the truncated cone.
    * 
    * @return the truncated cone's height.
    */
   public double getHeight()
   {
      return height;
   }

   /**
    * Returns the radius for the top face along the x-axis.
    * 
    * @return the radius for the top face along the x-axis.
    */
   public double getTopRadiusX()
   {
      return topRadiusX;
   }

   /**
    * Returns the radius for the top face along the y-axis.
    * 
    * @return the radius for the top face along the y-axis.
    */
   public double getTopRadiusY()
   {
      return topRadiusY;
   }

   /**
    * Returns the radius for the bottom face along the x-axis.
    * 
    * @return the radius for the bottom face along the x-axis.
    */
   public double getBaseRadiusX()
   {
      return baseRadiusX;
   }

   /**
    * Returns the radius for the bottom face along the y-axis.
    * 
    * @return the radius for the bottom face along the y-axis.
    */
   public double getBaseRadiusY()
   {
      return baseRadiusY;
   }

   /**
    * Returns whether the truncated cone should be centered at the origin.
    * 
    * @return {@code true} if the truncated cone should be centered at the origin, {@code false} if its
    *         bottom face should centered at the origin.
    */
   public boolean getCentered()
   {
      return centered;
   }

   /**
    * Returns the truncated cone's resolution which can be used for discretizing it.
    * 
    * @return the cone's resolution.
    */
   public int getResolution()
   {
      return resolution;
   }

   @Override
   public TruncatedCone3DDefinition copy()
   {
      return new TruncatedCone3DDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = super.hashCode();
      bits = EuclidHashCodeTools.addToHashCode(bits, height);
      bits = EuclidHashCodeTools.addToHashCode(bits, topRadiusX);
      bits = EuclidHashCodeTools.addToHashCode(bits, topRadiusY);
      bits = EuclidHashCodeTools.addToHashCode(bits, baseRadiusX);
      bits = EuclidHashCodeTools.addToHashCode(bits, baseRadiusY);
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

      TruncatedCone3DDefinition other = (TruncatedCone3DDefinition) object;

      if (Double.doubleToLongBits(height) != Double.doubleToLongBits(other.height))
         return false;
      if (Double.doubleToLongBits(topRadiusX) != Double.doubleToLongBits(other.topRadiusX))
         return false;
      if (Double.doubleToLongBits(topRadiusY) != Double.doubleToLongBits(other.topRadiusY))
         return false;
      if (Double.doubleToLongBits(baseRadiusX) != Double.doubleToLongBits(other.baseRadiusX))
         return false;
      if (Double.doubleToLongBits(baseRadiusY) != Double.doubleToLongBits(other.baseRadiusY))
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
      return "Truncated Cone: [name: " + getName() + ", height: " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, height)
            + EuclidCoreIOTools.getStringOf(", top radii: (", ")", ", ", topRadiusX, topRadiusY)
            + EuclidCoreIOTools.getStringOf(", base radii: (", ")", ", ", baseRadiusX, baseRadiusY) + ", resolution: " + resolution + "]";
   }
}
