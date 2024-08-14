package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.List;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
@XmlType(propOrder = {"name", "origin", "geometry"})
public class URDFCollision implements URDFItem
{
   private String name;
   private URDFOrigin origin;
   private URDFGeometry geometry;

   @XmlElement(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement(name = "origin")
   public void setOrigin(URDFOrigin origin)
   {
      this.origin = origin;
   }

   @XmlElement(name = "geometry")
   public void setGeometry(URDFGeometry geometry)
   {
      this.geometry = geometry;
   }

   public String getName()
   {
      return name;
   }

   public URDFOrigin getOrigin()
   {
      return origin;
   }

   public URDFGeometry getGeometry()
   {
      return geometry;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, origin: %s, geometry: %s]", name, origin, geometry);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemFilenameHolders(origin, geometry);
   }
}
