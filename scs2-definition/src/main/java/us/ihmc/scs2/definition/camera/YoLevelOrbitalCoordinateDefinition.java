package us.ihmc.scs2.definition.camera;

import jakarta.xml.bind.annotation.XmlElement;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.yoComposite.YoCompositeDefinition;

/**
 * Template for creating coordinates for positioning and orienting the 3D camera while allowing to
 * use components that can be backed by {@code YoVariable}s.
 */
public class YoLevelOrbitalCoordinateDefinition extends YoCompositeDefinition
{
   public static final String YoLevelOrbital = "YoLevelOrbitalCoordinate";
   public static final String[] YoLevelOrbitalIdentifiers = new String[] {"distance", "longitude", "height"};

   /**
    * The orbital coordinates.
    * <p>
    * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
    * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
    * to a constant value by using for instance {@link Double#toString(double)}.
    * </p>
    */
   private String distance, longitude, height;
   /**
    * The name id ({@link ReferenceFrame#getNameId()}) of the reference frame these coordinates are
    * expressed in, or {@code null} if they are expressed in world frame.
    */
   private String referenceFrame;

   /** Creates an empty definition which components need to be initialized. */
   public YoLevelOrbitalCoordinateDefinition()
   {
   }

   /**
    * Creates a new set of orbital coordinates that are expressed in world frame.
    *
    * @param distance  the constant value representation or {@code YoVariable} name/fullname for the
    *                  distance component.
    * @param longitude the constant value representation or {@code YoVariable} name/fullname for the
    *                  longitude component.
    * @param height    the constant value representation or {@code YoVariable} name/fullname for the
    *                  height component.
    */
   public YoLevelOrbitalCoordinateDefinition(String distance, String longitude, String height)
   {
      this(distance, longitude, height, null);
   }

   /**
    * Creates a new set of orbital coordinates that are expressed in world frame.
    *
    * @param distance       the constant value representation or {@code YoVariable} name/fullname for
    *                       the distance component.
    * @param longitude      the constant value representation or {@code YoVariable} name/fullname for
    *                       the longitude component.
    * @param height         the constant value representation or {@code YoVariable} name/fullname for
    *                       the height component.
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in
    *                       which the coordinates are to be expressed. Note that not all reference
    *                       frames are available from inside SCS2.
    */
   public YoLevelOrbitalCoordinateDefinition(String distance, String longitude, String height, String referenceFrame)
   {
      this.distance = distance;
      this.longitude = longitude;
      this.height = height;
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
    * Sets the height component to a constant double value.
    *
    * @param height the constant value for height.
    */
   public void setHeight(double height)
   {
      this.height = Double.toString(height);
   }

   /**
    * Sets the information for backing the height component.
    *
    * @param height the constant value representation or {@code YoVariable} name/fullname for the
    *               height component.
    */
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
      return YoLevelOrbital;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoLevelOrbitalIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {distance, longitude, height};
   }

   /**
    * Parses the given {@code value} into a {@link YoLevelOrbitalCoordinateDefinition}. The given
    * {@code String} representation is expected to have been generated using {@link #toString()}. If
    * the format differs, this method will throw an {code IllegalArgumentException}.
    *
    * @param value the {@code String} representation of a {@link YoLevelOrbitalCoordinateDefinition}.
    * @return the parsed orbital coordinates object.
    */
   public static YoLevelOrbitalCoordinateDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoLevelOrbital))
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

         return new YoLevelOrbitalCoordinateDefinition(distance, longitude, height, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown format: " + value);
      }
   }
}
