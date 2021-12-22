package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFInertial implements URDFItem
{
   private URDFOrigin origin;
   private URDFMass mass;
   private URDFInertia inertia;

   @XmlElement(name = "origin")
   public void setOrigin(URDFOrigin origin)
   {
      this.origin = origin;
   }

   @XmlElement(name = "mass")
   public void setMass(URDFMass mass)
   {
      this.mass = mass;
   }

   @XmlElement(name = "inertia")
   public void setInertia(URDFInertia inertia)
   {
      this.inertia = inertia;
   }

   public URDFOrigin getOrigin()
   {
      return origin;
   }

   public URDFMass getMass()
   {
      return mass;
   }

   public URDFInertia getInertia()
   {
      return inertia;
   }

   @Override
   public String getContentAsString()
   {
      return format("[origin: %s, mass: %s, inertia: %s.]", origin, mass, inertia);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return URDFItem.combineItemFilenameHolders(origin, mass, inertia);
   }
}
