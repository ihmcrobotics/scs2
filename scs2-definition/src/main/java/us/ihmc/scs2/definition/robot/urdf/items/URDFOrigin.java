package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFOrigin implements URDFItem
{
   private String xyz;
   private String rpy;

   @XmlAttribute(name = "xyz")
   public void setXYZ(String xyz)
   {
      this.xyz = xyz;
   }

   @XmlAttribute(name = "rpy")
   public void setRPY(String rpy)
   {
      this.rpy = rpy;
   }

   public String getXYZ()
   {
      return xyz;
   }

   public String getRPY()
   {
      return rpy;
   }

   @Override
   public String getContentAsString()
   {
      return format("[xyz: %s, rpy: %s.]", xyz, rpy);
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
