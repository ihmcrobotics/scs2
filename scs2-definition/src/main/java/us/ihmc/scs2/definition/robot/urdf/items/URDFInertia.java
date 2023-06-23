package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFInertia implements URDFItem
{
   private String ixx;
   private String iyy;
   private String izz;
   private String ixy;
   private String ixz;
   private String iyz;

   @XmlAttribute(name = "ixx")
   public void setIxx(String ixx)
   {
      this.ixx = ixx;
   }

   public void setIxx(double ixx)
   {
      setIxx(Double.toString(ixx));
   }

   @XmlAttribute(name = "iyy")
   public void setIyy(String iyy)
   {
      this.iyy = iyy;
   }

   public void setIyy(double iyy)
   {
      setIyy(Double.toString(iyy));
   }

   @XmlAttribute(name = "izz")
   public void setIzz(String izz)
   {
      this.izz = izz;
   }

   public void setIzz(double izz)
   {
      setIzz(Double.toString(izz));
   }

   @XmlAttribute(name = "ixy")
   public void setIxy(String ixy)
   {
      this.ixy = ixy;
   }

   public void setIxy(double ixy)
   {
      setIxy(Double.toString(ixy));
   }

   @XmlAttribute(name = "ixz")
   public void setIxz(String ixz)
   {
      this.ixz = ixz;
   }

   public void setIxz(double ixz)
   {
      setIxz(Double.toString(ixz));
   }

   @XmlAttribute(name = "iyz")
   public void setIyz(String iyz)
   {
      this.iyz = iyz;
   }

   public void setIyz(double iyz)
   {
      setIyz(Double.toString(iyz));
   }

   public String getIxx()
   {
      return ixx;
   }

   public String getIyy()
   {
      return iyy;
   }

   public String getIzz()
   {
      return izz;
   }

   public String getIxy()
   {
      return ixy;
   }

   public String getIxz()
   {
      return ixz;
   }

   public String getIyz()
   {
      return iyz;
   }

   @Override
   public String getContentAsString()
   {
      return format("[ixx: %s, iyy: %s, izz: %s, ixy: %s, ixz: %s, iyz: %s.]", ixx, iyy, izz, ixy, ixz, iyz);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return Collections.emptyList();
   }
}
