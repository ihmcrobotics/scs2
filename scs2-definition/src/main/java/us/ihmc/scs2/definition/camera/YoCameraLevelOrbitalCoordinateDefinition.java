package us.ihmc.scs2.definition.camera;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.scs2.definition.yoComposite.YoCompositeDefinition;

public class YoCameraLevelOrbitalCoordinateDefinition extends YoCompositeDefinition
{
   public static final String YoCameraLevelOrbital = "YoCameraLevelOrbitalCoordinate";
   public static final String[] YoCameraLevelOrbitalIdentifiers = new String[] {"distance", "longitude", "height"};

   private String distance, longitude, height;
   private String referenceFrame;

   public YoCameraLevelOrbitalCoordinateDefinition()
   {
   }

   public YoCameraLevelOrbitalCoordinateDefinition(String distance, String longitude, String height)
   {
      this(distance, longitude, height, null);
   }

   public YoCameraLevelOrbitalCoordinateDefinition(String distance, String longitude, String height, String referenceFrame)
   {
      this.distance = distance;
      this.longitude = longitude;
      this.height = height;
      this.referenceFrame = referenceFrame;
   }

   public void setDistance(double distance)
   {
      this.distance = Double.toString(distance);
   }

   @XmlElement
   public void setDistance(String distance)
   {
      this.distance = distance;
   }

   public void setLongitude(double longitude)
   {
      this.longitude = Double.toString(longitude);
   }

   @XmlElement
   public void setLongitude(String longitude)
   {
      this.longitude = longitude;
   }

   public void setHeight(double height)
   {
      this.height = Double.toString(height);
   }

   @XmlElement
   public void setHeight(String height)
   {
      this.height = height;
   }

   @XmlElement
   @Override
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public String getDistance()
   {
      return distance;
   }

   public String getLongitude()
   {
      return longitude;
   }

   public String getHeight()
   {
      return height;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoCameraLevelOrbital;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoCameraLevelOrbitalIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {distance, longitude, height};
   }

   public static YoCameraLevelOrbitalCoordinateDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoCameraLevelOrbital))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String distance = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String longitude = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String height = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (distance.equalsIgnoreCase("null"))
            distance = null;
         if (longitude.equalsIgnoreCase("null"))
            longitude = null;
         if (height.equalsIgnoreCase("null"))
            height = null;
         if (frame.equalsIgnoreCase("null"))
            frame = null;

         return new YoCameraLevelOrbitalCoordinateDefinition(distance, longitude, height, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown format: " + value);
      }
   }
}
