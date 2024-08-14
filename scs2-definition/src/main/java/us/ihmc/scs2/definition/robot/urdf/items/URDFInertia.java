package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

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

   @XmlAttribute(name = "iyy")
   public void setIyy(String iyy)
   {
      this.iyy = iyy;
   }

   @XmlAttribute(name = "izz")
   public void setIzz(String izz)
   {
      this.izz = izz;
   }

   @XmlAttribute(name = "ixy")
   public void setIxy(String ixy)
   {
      this.ixy = ixy;
   }

   @XmlAttribute(name = "ixz")
   public void setIxz(String ixz)
   {
      this.ixz = ixz;
   }

   @XmlAttribute(name = "iyz")
   public void setIyz(String iyz)
   {
      this.iyz = iyz;
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
