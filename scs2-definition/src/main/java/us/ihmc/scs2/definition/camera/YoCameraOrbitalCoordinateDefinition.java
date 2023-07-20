package us.ihmc.scs2.definition.camera;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.scs2.definition.yoComposite.YoCompositeDefinition;

public class YoCameraOrbitalCoordinateDefinition extends YoCompositeDefinition
{
   public static final String YoCameraOrbital = "YoCameraOrbitalCoordinate";
   public static final String[] YoCameraOrbitalIdentifiers = new String[] {"distance", "longitude", "latitude"};

   private String distance, longitude, latitude;
   private String referenceFrame;

   public YoCameraOrbitalCoordinateDefinition()
   {
   }

   public YoCameraOrbitalCoordinateDefinition(String distance, String longitude, String latitude)
   {
      this(distance, longitude, latitude, null);
   }

   public YoCameraOrbitalCoordinateDefinition(String distance, String longitude, String latitude, String referenceFrame)
   {
      this.distance = distance;
      this.longitude = longitude;
      this.latitude = latitude;
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

   public void setLatitude(double latitude)
   {
      this.latitude = Double.toString(latitude);
   }

   @XmlElement
   public void setLatitude(String latitude)
   {
      this.latitude = latitude;
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

   public String getLatitude()
   {
      return latitude;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoCameraOrbital;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoCameraOrbitalIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {distance, longitude, latitude};
   }

   public static YoCameraOrbitalCoordinateDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoCameraOrbital))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String distance = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String longitude = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String latitude = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (distance.equalsIgnoreCase("null"))
            distance = null;
         if (longitude.equalsIgnoreCase("null"))
            longitude = null;
         if (latitude.equalsIgnoreCase("null"))
            latitude = null;
         if (frame.equalsIgnoreCase("null"))
            frame = null;

         return new YoCameraOrbitalCoordinateDefinition(distance, longitude, latitude, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown format: " + value);
      }
   }
}
