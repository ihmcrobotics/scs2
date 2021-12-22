package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFCalibration implements URDFItem
{
   private String rising;
   private String falling;

   @XmlElement(name = "rising")
   public void setRising(String rising)
   {
      this.rising = rising;
   }

   @XmlElement(name = "falling")
   public void setFalling(String falling)
   {
      this.falling = falling;
   }

   public String getRising()
   {
      return rising;
   }

   public String getFalling()
   {
      return falling;
   }

   @Override
   public String getContentAsString()
   {
      return format("[rising: %s, falling: %s]", rising, falling);
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
