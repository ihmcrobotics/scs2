package us.ihmc.scs2.definition.robot.urdf.items;

import java.util.Collections;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;

/**
 * <a href="http://wiki.ros.org/urdf/XML/link"> ROS Specification link.</a>
 *
 * @author Sylvain Bertrand
 */
public class URDFMesh implements URDFItem, URDFFilenameHolder
{
   private String filename;
   private String scale;

   @Override
   @XmlAttribute(name = "filename")
   public void setFilename(String filename)
   {
      this.filename = filename;
   }

   @XmlAttribute(name = "scale")
   public void setScale(String scale)
   {
      this.scale = scale;
   }

   @Override
   public String getFilename()
   {
      return filename;
   }

   public String getScale()
   {
      return scale;
   }

   @Override
   public String getContentAsString()
   {
      return format("[filename: %s, scale: %s]", filename, scale);
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
