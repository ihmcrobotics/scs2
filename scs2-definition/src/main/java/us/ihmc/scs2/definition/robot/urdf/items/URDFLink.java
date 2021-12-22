package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFLink implements URDFItem
{
   private String name;
   private URDFInertial inertial;
   private List<URDFVisual> visuals;
   private List<URDFCollision> collisions;

   @XmlAttribute(name = "name")
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement(name = "inertial")
   public void setInertial(URDFInertial inertial)
   {
      this.inertial = inertial;
   }

   @XmlElement(name = "visual")
   public void setVisual(List<URDFVisual> visual)
   {
      this.visuals = visual;
   }

   @XmlElement(name = "collision")
   public void setCollision(List<URDFCollision> collision)
   {
      this.collisions = collision;
   }

   public String getName()
   {
      return name;
   }

   public URDFInertial getInertial()
   {
      return inertial;
   }

   public List<URDFVisual> getVisual()
   {
      return visuals;
   }

   public List<URDFCollision> getCollision()
   {
      return collisions;
   }

   @Override
   public String getContentAsString()
   {
      return format("[name: %s, inertial: %s, visual: %s, collision: %s]", name, inertial, visuals, collisions);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemListsFilenameHolders(Collections.singletonList(inertial), visuals, collisions);
   }
}
