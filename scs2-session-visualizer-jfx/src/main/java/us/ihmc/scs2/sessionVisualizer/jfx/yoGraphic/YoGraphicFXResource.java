package us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic;

import java.net.URL;

public class YoGraphicFXResource
{
   private final String resourceName;
   private final URL resourceURL;

   public YoGraphicFXResource(String resourceName)
   {
      this(resourceName, null);
   }

   public YoGraphicFXResource(String resourceName, URL resourceURL)
   {
      this.resourceName = resourceName;
      this.resourceURL = resourceURL;
   }

   public String getResourceName()
   {
      return resourceName;
   }

   public URL getResourceURL()
   {
      return resourceURL;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoGraphicFXResource)
      {
         YoGraphicFXResource other = (YoGraphicFXResource) object;
         if (resourceName == null ? other.resourceName != null : !resourceName.equals(other.resourceName))
            return false;
         if (resourceURL == null ? other.resourceURL != null : !resourceURL.equals(other.resourceURL))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "YoGraphicFXResource [resourceName=" + resourceName + ", resourceURL=" + resourceURL + "]";
   }
}
