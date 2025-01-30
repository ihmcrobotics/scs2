package us.ihmc.scs2.definition.yoComposite;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.Collections;
import java.util.List;

/**
 * A {@link YoQuaternionDefinition} represents a template for creating a quaternion which components
 * can be backed by {@code YoVariable}s.
 * <p>
 * A quaternion is used to represent a 3D orientation.
 * </p>
 * <p>
 * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
 * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
 * to a constant value by using for instance {@link Double#toString(double)}.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods to facilitate the creation of a
 * {@code YoQuaternionDefinition}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoQuaternion")
public class YoQuaternionDefinition extends YoOrientation3DDefinition
{
   public static final String YoQuaternion = "YoQuaternion";
   public static final String[] YoQuaternionIdentifiers = new String[] {"qx", "qy", "qz", "qs"};
   public static final List<String[]> YoQuaternionAlternateIdentifiers = Collections.singletonList(new String[] {"x, y, z, w"});

   /**
    * The components of this quaternion.
    * <p>
    * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
    * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
    * to a constant value by using for instance {@link Double#toString(double)}.
    * </p>
    * <p>
    * The component naming follows the Euclid convention, see {@link Quaternion}.
    * </p>
    */
   private String x, y, z, s;
   /**
    * The name id ({@link ReferenceFrame#getNameId()}) of the reference frame this quaternion is
    * expressed in, or {@code null} if it is expressed in world frame.
    */
   private String referenceFrame;

   /**
    * Creates an empty quaternion which components need to be initialized.
    */
   public YoQuaternionDefinition()
   {
   }

   /**
    * Creates a new quaternion that is expressed in world frame.
    *
    * @param x the constant value representation or {@code YoVariable} name/fullname for the
    *          x-component.
    * @param y the constant value representation or {@code YoVariable} name/fullname for the
    *          y-component.
    * @param z the constant value representation or {@code YoVariable} name/fullname for the
    *          z-component.
    * @param s the constant value representation or {@code YoVariable} name/fullname for the
    *          s-component.
    */
   public YoQuaternionDefinition(String x, String y, String z, String s)
   {
      this(x, y, z, s, null);
   }

   /**
    * Creates a new quaternion that is expressed in a specific frame.
    *
    * @param x              the constant value representation or {@code YoVariable} name/fullname for
    *                       the x-component.
    * @param y              the constant value representation or {@code YoVariable} name/fullname for
    *                       the y-component.
    * @param z              the constant value representation or {@code YoVariable} name/fullname for
    *                       the z-component.
    * @param s              the constant value representation or {@code YoVariable} name/fullname for
    *                       the s-component.
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in
    *                       which this quaternion is to be expressed. Note that not all reference
    *                       frames are available from inside SCS2.
    */
   public YoQuaternionDefinition(String x, String y, String z, String s, String referenceFrame)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.s = s;
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the x-component to a constant double value.
    *
    * @param x the constant value for x.
    */
   public void setX(double x)
   {
      this.x = Double.toString(x);
   }

   /**
    * Sets the information for backing the x-component.
    *
    * @param x the constant value representation or {@code YoVariable} name/fullname for the
    *          x-component.
    */
   @XmlElement
   public void setX(String x)
   {
      this.x = x;
   }

   /**
    * Sets the y-component to a constant double value.
    *
    * @param y the constant value for y.
    */
   public void setY(double y)
   {
      this.y = Double.toString(y);
   }

   /**
    * Sets the information for backing the y-component.
    *
    * @param y the constant value representation or {@code YoVariable} name/fullname for the
    *          y-component.
    */
   @XmlElement
   public void setY(String y)
   {
      this.y = y;
   }

   /**
    * Sets the z-component to a constant double value.
    *
    * @param z the constant value for z.
    */
   public void setZ(double z)
   {
      this.z = Double.toString(z);
   }

   /**
    * Sets the information for backing the z-component.
    *
    * @param z the constant value representation or {@code YoVariable} name/fullname for the
    *          z-component.
    */
   @XmlElement
   public void setZ(String z)
   {
      this.z = z;
   }

   /**
    * Sets the s-component to a constant double value.
    *
    * @param s the constant value for s.
    */
   public void setS(double s)
   {
      this.s = Double.toString(s);
   }

   /**
    * Sets the information for backing the s-component.
    *
    * @param s the constant value representation or {@code YoVariable} name/fullname for the
    *          s-component.
    */
   @XmlElement
   public void setS(String s)
   {
      this.s = s;
   }

   @XmlElement
   @Override
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /**
    * Returns the constant value representation or {@code YoVariable} name/fullname for the
    * x-component.
    *
    * @return the x-component.
    */
   public String getX()
   {
      return x;
   }

   /**
    * Returns the constant value representation or {@code YoVariable} name/fullname for the
    * y-component.
    *
    * @return the y-component.
    */
   public String getY()
   {
      return y;
   }

   /**
    * Returns the constant value representation or {@code YoVariable} name/fullname for the
    * z-component.
    *
    * @return the z-component.
    */
   public String getZ()
   {
      return z;
   }

   /**
    * Returns the constant value representation or {@code YoVariable} name/fullname for the
    * s-component.
    *
    * @return the s-component.
    */
   public String getS()
   {
      return s;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoQuaternion;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoQuaternionIdentifiers;
   }

   @Override
   public List<String[]> getAlternateComponentIdentifiers()
   {
      return YoQuaternionAlternateIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {x, y, z, s};
   }

   @Override
   public YoQuaternionDefinition copy()
   {
      return new YoQuaternionDefinition(x, y, z, s, referenceFrame);
   }

   /**
    * Parses the given {@code value} into a {@link YoQuaternionDefinition}. The given {@code String}
    * representation is expected to have been generated using {@link #toString()}. If the format
    * differs, this method will throw an {code IllegalArgumentException}.
    *
    * @param value the {@code String} representation of a {@link YoQuaternionDefinition}.
    * @return the parsed quaternion object.
    */
   public static YoQuaternionDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoQuaternion))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String x = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String y = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String z = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String s = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (x.equalsIgnoreCase("null"))
            x = null;
         if (y.equalsIgnoreCase("null"))
            y = null;
         if (z.equalsIgnoreCase("null"))
            z = null;
         if (s.equalsIgnoreCase("null"))
            s = null;
         if (frame.equalsIgnoreCase("null"))
            frame = null;

         return new YoQuaternionDefinition(x, y, z, s, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown quaternion format: " + value);
      }
   }
}
