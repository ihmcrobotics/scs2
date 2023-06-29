package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
@XmlType(propOrder = {"name", "origin", "geometry", "material"})
public class URDFVisual implements URDFItem
{
   private String name;
   private URDFOrigin origin;
   private URDFGeometry geometry;
   private URDFMaterial material;

   @XmlAttribute(name = "name")
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

   @XmlElement(name = "material")
   public void setMaterial(URDFMaterial material)
   {
      this.material = material;
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

   public URDFMaterial getMaterial()
   {
      return material;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, origin: %s, geometry: %s, material: %s]", name, origin, geometry, material);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemFilenameHolders(origin, geometry, material);
   }
}
