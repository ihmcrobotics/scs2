package us.ihmc.scs2.definition.robot.urdf.items;

import jakarta.xml.bind.annotation.XmlAttribute;

import java.util.Collections;
import java.util.List;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFCapsule implements URDFItem
{
   private String length;
   private String radius;

   @XmlAttribute(name = "length")
   public void setLength(String length)
   {
      this.length = length;
   }

   @XmlAttribute(name = "radius")
   public void setRadius(String radius)
   {
      this.radius = radius;
   }

   public String getLength()
   {
      return length;
   }

   public String getRadius()
   {
      return radius;
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
