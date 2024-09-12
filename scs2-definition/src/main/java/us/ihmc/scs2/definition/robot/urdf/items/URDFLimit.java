package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFLimit implements URDFItem
{
   private String lower;
   private String upper;
   private String effort;
   private String velocity;

   @XmlAttribute(name = "lower")
   public void setLower(String lower)
   {
      this.lower = lower;
   }

   @XmlAttribute(name = "upper")
   public void setUpper(String upper)
   {
      this.upper = upper;
   }

   @XmlAttribute(name = "effort")
   public void setEffort(String effort)
   {
      this.effort = effort;
   }

   @XmlAttribute(name = "velocity")
   public void setVelocity(String velocity)
   {
      this.velocity = velocity;
   }

   public String getLower()
   {
      return lower;
   }

   public String getUpper()
   {
      return upper;
   }

   public String getEffort()
   {
      return effort;
   }

   public String getVelocity()
   {
      return velocity;
   }

   @Override
   public String getContentAsString()
   {
      return format("lower: %s, upper: %s, effort: %s, velocity: %s]", lower, upper, effort, velocity);
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
