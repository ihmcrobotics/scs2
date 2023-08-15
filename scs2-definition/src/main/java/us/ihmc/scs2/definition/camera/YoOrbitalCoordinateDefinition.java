package us.ihmc.scs2.definition.camera;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.yoComposite.YoCompositeDefinition;

/**
 * Template for creating coordinates for positioning and orienting the 3D camera while allowing to
 * use components that can be backed by {@code YoVariable}s.
 * 
 * @author Sylvain Bertrand
 */
public class YoOrbitalCoordinateDefinition extends YoCompositeDefinition
{
   public static final String YoOrbital = "YoOrbitalCoordinate";
   public static final String[] YoOrbitalIdentifiers = new String[] {"distance", "longitude", "latitude"};

   /**
    * The orbital coordinates.
    * <p>
    * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
    * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
    * to a constant value by using for instance {@link Double#toString(double)}.
    * </p>
    */
   private String distance, longitude, latitude;
   /**
    * The name id ({@link ReferenceFrame#getNameId()}) of the reference frame these coordinates are
    * expressed in, or {@code null} if they are expressed in world frame.
    */
   private String referenceFrame;

   /** Creates an empty definition which components need to be initialized. */
   public YoOrbitalCoordinateDefinition()
   {
   }

   /**
    * Creates a new set of orbital coordinates that are expressed in world frame.
    *
    * @param distance  the constant value representation or {@code YoVariable} name/fullname for the
    *                  distance component.
    * @param longitude the constant value representation or {@code YoVariable} name/fullname for the
    *                  longitude component.
    * @param latitude  the constant value representation or {@code YoVariable} name/fullname for the
    *                  latitude component.
    */
   public YoOrbitalCoordinateDefinition(String distance, String longitude, String latitude)
   {
      this(distance, longitude, latitude, null);
   }

   /**
    * Creates a new set of orbital coordinates that are expressed in world frame.
    *
    * @param distance       the constant value representation or {@code YoVariable} name/fullname for
    *                       the distance component.
    * @param longitude      the constant value representation or {@code YoVariable} name/fullname for
    *                       the longitude component.
    * @param latitude       the constant value representation or {@code YoVariable} name/fullname for
    *                       the latitude component.
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in
    *                       which the coordinates are to be expressed. Note that not all reference
    *                       frames are available from inside SCS2.
    */
   public YoOrbitalCoordinateDefinition(String distance, String longitude, String latitude, String referenceFrame)
   {
      this.distance = distance;
      this.longitude = longitude;
      this.latitude = latitude;
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the distance component to a constant double value.
    *
    * @param distance the constant value for distance.
    */
   public void setDistance(double distance)
   {
      this.distance = Double.toString(distance);
   }

   /**
    * Sets the information for backing the distance component.
    *
    * @param distance the constant value representation or {@code YoVariable} name/fullname for the
    *                 distance component.
    */
   @XmlElement
   public void setDistance(String distance)
   {
      this.distance = distance;
   }

   /**
    * Sets the longitude component to a constant double value.
    *
    * @param longtude the constant value for longitude.
    */
   public void setLongitude(double longitude)
   {
      this.longitude = Double.toString(longitude);
   }

   /**
    * Sets the information for backing the longitude component.
    *
    * @param longitude the constant value representation or {@code YoVariable} name/fullname for the
    *                  longitude component.
    */
   @XmlElement
   public void setLongitude(String longitude)
   {
      this.longitude = longitude;
   }

   /**
    * Sets the latitude component to a constant double value.
    *
    * @param latitude the constant value for latitude.
    */
   public void setLatitude(double latitude)
   {
      this.latitude = Double.toString(latitude);
   }

   /**
    * Sets the information for backing the latitude component.
    *
    * @param latitude the constant value representation or {@code YoVariable} name/fullname for the
    *                 latitude component.
    */
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
      return YoOrbital;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoOrbitalIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {distance, longitude, latitude};
   }

   /**
    * Parses the given {@code value} into a {@link YoOrbitalCoordinateDefinition}. The given
    * {@code String} representation is expected to have been generated using {@link #toString()}. If
    * the format differs, this method will throw an {code IllegalArgumentException}.
    *
    * @param value the {@code String} representation of a {@link YoOrbitalCoordinateDefinition}.
    * @return the parsed orbital coordinates object.
    */
   public static YoOrbitalCoordinateDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoOrbital))
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

         return new YoOrbitalCoordinateDefinition(distance, longitude, latitude, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown format: " + value);
      }
   }
}
