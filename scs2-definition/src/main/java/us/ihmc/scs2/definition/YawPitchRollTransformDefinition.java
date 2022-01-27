package us.ihmc.scs2.definition;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

@XmlType(propOrder = {"x", "y", "z", "yaw", "pitch", "roll"})
public class YawPitchRollTransformDefinition implements RigidBodyTransformBasics
{
   private final Vector3D translation = new Vector3D();
   private final YawPitchRoll orientation = new YawPitchRoll();

   public YawPitchRollTransformDefinition()
   {
   }

   public YawPitchRollTransformDefinition(double x, double y, double z)
   {
      setTranslation(x, y, z);
   }

   public YawPitchRollTransformDefinition(double x, double y, double z, double yaw, double pitch, double roll)
   {
      setTranslation(x, y, z);
      setOrientation(yaw, pitch, roll);
   }

   public YawPitchRollTransformDefinition(Tuple3DReadOnly translation)
   {
      setTranslation(translation);
   }

   public YawPitchRollTransformDefinition(Orientation3DReadOnly orientation)
   {
      setTranslation(translation);
      setOrientation(orientation);
   }

   public YawPitchRollTransformDefinition(Tuple3DReadOnly translation, Orientation3DReadOnly orientation)
   {
      setTranslation(translation);
      setOrientation(orientation);
   }

   public YawPitchRollTransformDefinition(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   @Override
   public void setTranslation(double x, double y, double z)
   {
      translation.set(x, y, z);
   }

   @Override
   public void setTranslation(Tuple3DReadOnly translation)
   {
      this.translation.set(translation);
   }

   public void setOrientation(double yaw, double pitch, double roll)
   {
      orientation.set(yaw, pitch, roll);
   }

   public void setOrientation(Orientation3DReadOnly orientation)
   {
      this.orientation.set(orientation);
   }

   @XmlAttribute
   public void setX(double x)
   {
      translation.setX(x);
   }

   @XmlAttribute
   public void setY(double y)
   {
      translation.setY(y);
   }

   @XmlAttribute
   public void setZ(double z)
   {
      translation.setZ(z);
   }

   @XmlAttribute
   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
   }

   @XmlAttribute
   public void setPitch(double pitch)
   {
      orientation.setPitch(pitch);
   }

   @XmlAttribute
   public void setRoll(double roll)
   {
      orientation.setRoll(roll);
   }

   public double getX()
   {
      return translation.getX();
   }

   public double getY()
   {
      return translation.getY();
   }

   public double getZ()
   {
      return translation.getZ();
   }

   public double getYaw()
   {
      return orientation.getYaw();
   }

   public double getPitch()
   {
      return orientation.getPitch();
   }

   public double getRoll()
   {
      return orientation.getRoll();
   }

   @Override
   public Vector3D getTranslation()
   {
      return translation;
   }

   @Override
   public Orientation3DBasics getRotation()
   {
      return orientation;
   }

   @Override
   public String toString()
   {
      return "[(x,y,z)=" + translation + ", (y,p,r)="
            + EuclidCoreIOTools.getStringOf("(", ")]", ", ", orientation.getYaw(), orientation.getPitch(), orientation.getRoll());
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((orientation == null) ? 0 : orientation.hashCode());
      result = prime * result + ((translation == null) ? 0 : translation.hashCode());
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      YawPitchRollTransformDefinition other = (YawPitchRollTransformDefinition) obj;
      if (orientation == null)
      {
         if (other.orientation != null)
            return false;
      }
      else if (!orientation.equals(other.orientation))
         return false;
      if (translation == null)
      {
         if (other.translation != null)
            return false;
      }
      else if (!translation.equals(other.translation))
         return false;
      return true;
   }
}
