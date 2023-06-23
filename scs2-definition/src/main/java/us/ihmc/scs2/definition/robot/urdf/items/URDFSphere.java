package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFSphere implements URDFItem
{
   private String radius;

   @XmlAttribute(name = "radius")
   public void setRadius(String radius)
   {
      this.radius = radius;
   }

   public void setRadius(double radius)
   {
      setRadius(Double.toString(radius));
   }

   public String getRadius()
   {
      return radius;
   }

   @Override
   public String getContentAsString()
   {
      return format("[radius: %s]", radius);
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
