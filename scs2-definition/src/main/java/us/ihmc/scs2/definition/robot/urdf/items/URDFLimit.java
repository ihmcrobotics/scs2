package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;

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

   public void setLower(double lower)
   {
      setLower(Double.toString(lower));
   }

   @XmlAttribute(name = "upper")
   public void setUpper(String upper)
   {
      this.upper = upper;
   }

   public void setUpper(double upper)
   {
      setUpper(Double.toString(upper));
   }

   @XmlAttribute(name = "effort")
   public void setEffort(String effort)
   {
      this.effort = effort;
   }

   public void setEffort(double effort)
   {
      setEffort(Double.toString(effort));
   }

   @XmlAttribute(name = "velocity")
   public void setVelocity(String velocity)
   {
      this.velocity = velocity;
   }

   public void setVelocity(double velocity)
   {
      setVelocity(Double.toString(velocity));
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
