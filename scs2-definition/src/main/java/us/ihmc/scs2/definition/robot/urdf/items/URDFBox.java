package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFBox implements URDFItem
{
   private String size;

   @XmlAttribute(name = "size")
   public void setSize(String size)
   {
      this.size = size;
   }

   public String getSize()
   {
      return size;
   }

   @Override
   public String getContentAsString()
   {
      return format("[size: %s]", size);
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
