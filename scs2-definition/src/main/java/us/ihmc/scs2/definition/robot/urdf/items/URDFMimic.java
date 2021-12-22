package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFMimic implements URDFItem
{
   private String joint;
   private String multiplier;
   private String offset;

   @XmlElement(name = "joint")
   public void setJoint(String joint)
   {
      this.joint = joint;
   }

   @XmlElement(name = "multiplier")
   public void setMultiplier(String multiplier)
   {
      this.multiplier = multiplier;
   }

   @XmlElement(name = "offset")
   public void setOffset(String offset)
   {
      this.offset = offset;
   }

   public String getJoint()
   {
      return joint;
   }

   public String getMultiplier()
   {
      return multiplier;
   }

   public String getOffset()
   {
      return offset;
   }

   @Override
   public String getContentAsString()
   {
      return format("[joint: %s, multiplier: %s, offset: %s]", joint, multiplier, offset);
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
