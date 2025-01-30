package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFColor implements URDFItem
{
   private String rgba;

   @XmlAttribute(name = "rgba")
   public void setRGBA(String rgba)
   {
      this.rgba = rgba;
   }

   public String getRGBA()
   {
      return rgba;
   }

   @Override
   public String getContentAsString()
   {
      return format("[rgba: %s]", rgba);
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
