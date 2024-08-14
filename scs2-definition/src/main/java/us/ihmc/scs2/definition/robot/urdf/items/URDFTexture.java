package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFTexture implements URDFItem, URDFFilenameHolder
{
   private String filename;

   @Override
   @XmlAttribute(name = "filename")
   public void setFilename(String filename)
   {
      this.filename = filename;
   }

   @Override
   public String getFilename()
   {
      return filename;
   }

   @Override
   public String getContentAsString()
   {
      return format("[filename: %s]", filename);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return Collections.singletonList(this);
   }
}
