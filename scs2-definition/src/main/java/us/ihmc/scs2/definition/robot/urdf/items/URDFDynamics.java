package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFDynamics implements URDFItem
{
   private String damping;
   private String friction;

   @XmlAttribute(name = "damping")
   public void setDamping(String damping)
   {
      this.damping = damping;
   }

   @XmlAttribute(name = "friction")
   public void setFriction(String friction)
   {
      this.friction = friction;
   }

   public String getDamping()
   {
      return damping;
   }

   public String getFriction()
   {
      return friction;
   }

   @Override
   public String getContentAsString()
   {
      return format("[damping: %s, friction: %s]", damping, friction);
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
