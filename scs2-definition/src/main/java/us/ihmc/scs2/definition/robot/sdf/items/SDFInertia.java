package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;

public class SDFInertia implements SDFItem
{
   private String ixx;
   private String ixy;
   private String ixz;
   private String iyy;
   private String iyz;
   private String izz;

   public String getIxx()
   {
      return ixx;
   }

   @XmlElement(name = "ixx")
   public void setIxx(String ixx)
   {
      this.ixx = ixx;
   }

   public String getIxy()
   {
      return ixy;
   }

   @XmlElement(name = "ixy")
   public void setIxy(String ixy)
   {
      this.ixy = ixy;
   }

   public String getIxz()
   {
      return ixz;
   }

   @XmlElement(name = "ixz")
   public void setIxz(String ixz)
   {
      this.ixz = ixz;
   }

   public String getIyy()
   {
      return iyy;
   }

   @XmlElement(name = "iyy")
   public void setIyy(String iyy)
   {
      this.iyy = iyy;
   }

   public String getIyz()
   {
      return iyz;
   }

   @XmlElement(name = "iyz")
   public void setIyz(String iyz)
   {
      this.iyz = iyz;
   }

   public String getIzz()
   {
      return izz;
   }

   @XmlElement(name = "izz")
   public void setIzz(String izz)
   {
      this.izz = izz;
   }

   @Override
   public String getContentAsString()
   {
      return format("[ixx: %s, ixy: %s, ixz: %s, iyy: %s, iyz: %s, izz: %s]", ixx, ixy, ixz, iyy, iyz, izz);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return Collections.emptyList();
   }

   @Override
   public String toString()
   {
      return itemToString();
   }
}