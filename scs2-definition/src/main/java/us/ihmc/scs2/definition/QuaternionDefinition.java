package us.ihmc.scs2.definition;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * Implementation for a quaternion that is compatible with the JAXB serialization.
 * <p>
 * Do not use for other applications, use {@link Quaternion} instead.
 * </p>
 */
public class QuaternionDefinition implements QuaternionBasics
{
   private double x;
   private double y;
   private double z;
   private double s;

   public QuaternionDefinition()
   {
      setToZero();
   }

   public void setX(double x)
   {
      this.x = x;
   }

   public void setY(double y)
   {
      this.y = y;
   }

   public void setZ(double z)
   {
      this.z = z;
   }

   public void setS(double s)
   {
      this.s = s;
   }

   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      setX(qx);
      setY(qy);
      setZ(qz);
      setS(qs);
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }

   @Override
   public double getS()
   {
      return s;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Tuple4DReadOnly)
         return equals((Tuple4DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(x, y, z, s);
   }
}
