package us.ihmc.scs2.definition.robot.urdf.items;

import jakarta.xml.bind.annotation.XmlAttribute;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

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

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof URDFLinkReference other)
      {
         return Objects.equals(link, other.link);
      }
      else
      {
         return false;
      }
   }
}