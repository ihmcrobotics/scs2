package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;

/**
 * A {@link YoYawPitchRollDefinition} represents a template for creating a yaw-pitch-roll which
 * components can be backed by {@code YoVariable}s.
 * <p>
 * A yaw-pitch-roll is used to represent a 3D orientation.
 * </p>
 * <p>
 * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
 * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
 * to a constant value by using for instance {@link Double#toString(double)}.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods to facilitate the creation of a
 * {@code YoYawPitchRollDefinition}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoYawPitchRoll")
public class YoYawPitchRollDefinition extends YoOrientation3DDefinition
{
   public static final String YoYawPitchRoll = "YoYawPitchRoll";
   public static final String[] YoYawPitchRollIdentifiers = new String[] {"yaw", "pitch", "roll"};

   /**
    * The components of this yaw-pitch-roll.
    * <p>
    * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
    * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
    * to a constant value by using for instance {@link Double#toString(double)}.
    * </p>
    */
   private String yaw, pitch, roll;
   /**
    * The name id ({@link ReferenceFrame#getNameId()}) of the reference frame this yaw-pitch-roll is
    * expressed in, or {@code null} if it is expressed in world frame.
    */
   private String referenceFrame;

   /** Creates an empty yaw-pitch-roll which components need to be initialized. */
   public YoYawPitchRollDefinition()
   {
   }

   /**
    * Creates a new yaw-pitch-roll that is expressed in world frame.
    *
    * @param yaw   the constant value representation or {@code YoVariable} name/fullname for the
    *              yaw-component.
    * @param pitch the constant value representation or {@code YoVariable} name/fullname for the
    *              pitch-component.
    * @param roll  the constant value representation or {@code YoVariable} name/fullname for the
    *              roll-component.
    */
   public YoYawPitchRollDefinition(String yaw, String pitch, String roll)
   {
      this(yaw, pitch, roll, null);
   }

   /**
    * Creates a new yaw-pitch-roll that is expressed in a specific frame.
    *
    * @param yaw            the constant value representation or {@code YoVariable} name/fullname for
    *                       the yaw-component.
    * @param pitch          the constant value representation or {@code YoVariable} name/fullname for
    *                       the pitch-component.
    * @param roll           the constant value representation or {@code YoVariable} name/fullname for
    *                       the roll-component.
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in
    *                       which this yaw-pitch-roll is to be expressed. Note that not all reference
    *                       frames are available from inside SCS2.
    */
   public YoYawPitchRollDefinition(String yaw, String pitch, String roll, String referenceFrame)
   {
      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the yaw-component to a constant double value.
    *
    * @param yaw the constant value for yaw.
    */
   public void setYaw(double yaw)
   {
      this.yaw = Double.toString(yaw);
   }

   /**
    * Sets the information for backing the x-component.
    *
    * @param yaw the constant value representation or {@code YoVariable} name/fullname for the
    *            yaw-component.
    */
   @XmlElement
   public void setYaw(String yaw)
   {
      this.yaw = yaw;
   }

   /**
    * Sets the pitch-component to a constant double value.
    *
    * @param pitch the constant value for pitch.
    */
   public void setPitch(double pitch)
   {
      this.pitch = Double.toString(pitch);
   }

   /**
    * Sets the information for backing the x-component.
    *
    * @param pitch the constant value representation or {@code YoVariable} name/fullname for the
    *              pitch-component.
    */
   @XmlElement
   public void setPitch(String pitch)
   {
      this.pitch = pitch;
   }

   /**
    * Sets the roll-component to a constant double value.
    *
    * @param roll the constant value for roll.
    */
   public void setRoll(double roll)
   {
      this.roll = Double.toString(roll);
   }

   /**
    * Sets the information for backing the x-component.
    *
    * @param roll the constant value representation or {@code YoVariable} name/fullname for the
    *             roll-component.
    */
   @XmlElement
   public void setRoll(String roll)
   {
      this.roll = roll;
   }

   @XmlElement
   @Override
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public String getYaw()
   {
      return yaw;
   }

   public String getPitch()
   {
      return pitch;
   }

   public String getRoll()
   {
      return roll;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoYawPitchRoll;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoYawPitchRollIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {yaw, pitch, roll};
   }

   /**
    * Parses the given {@code value} into a {@link YoYawPitchRollDefinition}. The given {@code String}
    * representation is expected to have been generated using {@link #toString()}. If the format
    * differs, this method will throw an {code IllegalArgumentException}.
    *
    * @param value the {@code String} representation of a {@link YoYawPitchRollDefinition}.
    * @return the parsed yaw-pitch-roll object.
    */
   public static YoYawPitchRollDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoYawPitchRoll))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String yaw = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String pitch = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String roll = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (yaw.equalsIgnoreCase("null"))
            yaw = null;
         if (pitch.equalsIgnoreCase("null"))
            pitch = null;
         if (roll.equalsIgnoreCase("null"))
            roll = null;
         if (frame.equalsIgnoreCase("null"))
            frame = null;

         return new YoYawPitchRollDefinition(yaw, pitch, roll, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown yaw-pitch-roll format: " + value);
      }
   }
}
