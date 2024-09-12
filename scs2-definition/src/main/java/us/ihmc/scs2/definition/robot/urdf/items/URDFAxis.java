package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFAxis implements URDFItem
{
   private String xyz;

   @XmlAttribute(name = "xyz")
   public void setXYZ(String xyz)
   {
      this.xyz = xyz;
   }

   public String getXYZ()
   {
      return xyz;
   }

   @Override
   public String getContentAsString()
   {
      return format("[xyz: %s]", xyz);
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
