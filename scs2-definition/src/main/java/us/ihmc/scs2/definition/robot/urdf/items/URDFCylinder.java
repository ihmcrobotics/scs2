package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFCylinder implements URDFItem
{
   private String radius;
   private String length;

   @XmlAttribute(name = "radius")
   public void setRadius(String radius)
   {
      this.radius = radius;
   }

   @XmlAttribute(name = "length")
   public void setLength(String length)
   {
      this.length = length;
   }

   public String getRadius()
   {
      return radius;
   }

   public String getLength()
   {
      return length;
   }

   @Override
   public String getContentAsString()
   {
      return format("[radius: %s, length: %s]", radius, length);
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
