package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/joint"> ROS Specification joint.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFLinkReference implements URDFItem
{
   private String link;

   @XmlAttribute(name = "link")
   public void setLink(String link)
   {
      this.link = link;
   }

   public String getLink()
   {
      return link;
   }

   @Override
   public String getContentAsString()
   {
      return format("[link: %s]", link);
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