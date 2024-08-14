package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFSafetyController implements URDFItem
{
   private String softLowerLimit;
   private String softUpperLimit;
   private String kPosition;
   private String kVelocity;

   @XmlElement(name = "soft_lower_limit")
   public void setSoftLowerLimit(String softLowerLimit)
   {
      this.softLowerLimit = softLowerLimit;
   }

   @XmlElement(name = "soft_upper_limit")
   public void setSoftUpperLimit(String softUpperLimit)
   {
      this.softUpperLimit = softUpperLimit;
   }

   @XmlElement(name = "k_position")
   public void setKPosition(String kPosition)
   {
      this.kPosition = kPosition;
   }

   @XmlElement(name = "k_velocity")
   public void setKVelocity(String kVelocity)
   {
      this.kVelocity = kVelocity;
   }

   public String getSoftLowerLimit()
   {
      return softLowerLimit;
   }

   public String getSoftUpperLimit()
   {
      return softUpperLimit;
   }

   public String getKPosition()
   {
      return kPosition;
   }

   public String getKVelocity()
   {
      return kVelocity;
   }

   @Override
   public String getContentAsString()
   {
      return format("[softLowerLimit: %s, softUpperLimit: %s, kPosition: %s, kVelocity: %s]", softLowerLimit, softUpperLimit, kPosition, kVelocity);
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
