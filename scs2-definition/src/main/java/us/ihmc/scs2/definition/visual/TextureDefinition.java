package us.ihmc.scs2.definition.visual;

import java.awt.image.BufferedImage;
import java.net.URL;
import java.util.Objects;

import us.ihmc.euclid.tools.EuclidHashCodeTools;
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
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, fileURL);
      bits = EuclidHashCodeTools.addToHashCode(bits, filename);
      bits = EuclidHashCodeTools.addToHashCode(bits, image);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      TextureDefinition other = (TextureDefinition) object;

      if (!Objects.equals(fileURL, other.fileURL))
         return false;
      if (!Objects.equals(filename, other.filename))
         return false;
      if (!Objects.equals(image, other.image))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "[fileURL=" + fileURL + ", filename=" + filename + ", image=" + image + "]";
   }
}