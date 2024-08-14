package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFMaterial implements URDFItem
{
   private String name;
   private URDFColor color;
   private URDFTexture texture;

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement(name = "color")
   public void setColor(URDFColor color)
   {
      this.color = color;
   }

   @XmlElement(name = "texture")
   public void setTexture(URDFTexture texture)
   {
      this.texture = texture;
   }

   public String getName()
   {
      return name;
   }

   public URDFColor getColor()
   {
      return color;
   }

   public URDFTexture getTexture()
   {
      return texture;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, color: %s, texture: %s.]", name, color, texture);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemFilenameHolders(color, texture);
   }
}
