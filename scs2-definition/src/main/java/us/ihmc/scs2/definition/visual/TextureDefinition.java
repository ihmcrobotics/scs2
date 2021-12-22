package us.ihmc.scs2.definition.visual;

import java.awt.image.BufferedImage;
import java.net.URL;

import us.ihmc.scs2.definition.geometry.ExtrusionDefinition;

public class TextureDefinition
{
   private URL fileURL;
   private String filename;
   private BufferedImage image;

   public TextureDefinition()
   {
   }

   public TextureDefinition(URL fileURL)
   {
      setFileURL(fileURL);
   }

   public TextureDefinition(String filename)
   {
      setFilename(filename);
   }

   public TextureDefinition(BufferedImage image)
   {
      setImage(image);
   }

   public TextureDefinition(TextureDefinition other)
   {
      set(other);
   }

   public void set(TextureDefinition other)
   {
      fileURL = other.fileURL;
      filename = other.filename;
      image = ExtrusionDefinition.copyImage(other.image);
   }

   public void setFileURL(URL fileURL)
   {
      this.fileURL = fileURL;
   }

   public void setFilename(String filename)
   {
      this.filename = filename;
   }

   public void setImage(BufferedImage image)
   {
      this.image = image;
   }

   public URL getFileURL()
   {
      return fileURL;
   }

   public String getFilename()
   {
      return filename;
   }

   public BufferedImage getImage()
   {
      return image;
   }

   public TextureDefinition copy()
   {
      return new TextureDefinition(this);
   }

   @Override
   public String toString()
   {
      return "[fileURL=" + fileURL + ", filename=" + filename + ", image=" + image + "]";
   }
}