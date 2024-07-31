package us.ihmc.scs2.definition.yoComposite;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

/**
 * A {@link YoTuple3DDefinition} represents a template for creating a tuple 3D which components can
 * be backed by {@code YoVariable}s.
 * <p>
 * A tuple 3D can be used to represent either a point or a vector.
 * </p>
 * <p>
 * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
 * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
 * to a constant value by using for instance {@link Double#toString(double)}.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods to facilitate the creation of a
 * {@code YoTuple3DDefinition}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoTuple3D")
public class YoTuple3DDefinition extends YoCompositeDefinition
{
   public static final String YoTuple3D = "YoTuple3D";
   public static final String[] YoTuple3DIdentifiers = new String[] {"x", "y", "z"};

   /**
    * The components of this tuple 3D.
    * <p>
    * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
    * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
    * to a constant value by using for instance {@link Double#toString(double)}.
    * </p>
    */
   private String x, y, z;
   /**
    * The name id ({@link ReferenceFrame#getNameId()}) of the reference frame this tuple 3D is
    * expressed in, or {@code null} if it is expressed in world frame.
    */
   private String referenceFrame;

   /** Creates an empty tuple 3D which components need to be initialized. */
   public YoTuple3DDefinition()
   {
   }

   /**
    * Creates a new tuple 3D that is expressed in world frame.
    *
    * @param x the constant value representation or {@code YoVariable} name/fullname for the
    *          x-component.
    * @param y the constant value representation or {@code YoVariable} name/fullname for the
    *          y-component.
    * @param z the constant value representation or {@code YoVariable} name/fullname for the
    *          z-component.
    */
   public YoTuple3DDefinition(String x, String y, String z)
   {
      this(x, y, z, null);
   }

   /**
    * Creates a new tuple 3D that is expressed in a specific frame.
    *
    * @param x              the constant value representation or {@code YoVariable} name/fullname for
    *                       the x-component.
    * @param y              the constant value representation or {@code YoVariable} name/fullname for
    *                       the y-component.
    * @param z              the constant value representation or {@code YoVariable} name/fullname for
    *                       the z-component.
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in
    *                       which this tuple 3D is to be expressed. Note that not all reference frames
    *                       are available from inside SCS2.
    */
   public YoTuple3DDefinition(String x, String y, String z, String referenceFrame)
   {
      this.x = x;
      this.y = y;
      this.z = z;
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

   @XmlElement
   @Override
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public String getX()
   {
      return x;
   }

   public String getY()
   {
      return y;
   }

   public String getZ()
   {
      return z;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoTuple3D;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoTuple3DIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {x, y, z};
   }

   public YoTuple3DDefinition copy()
   {
      return new YoTuple3DDefinition(x, y, z, referenceFrame);
   }

   /**
    * Parses the given {@code value} into a {@link YoTuple3DDefinition}. The given {@code String}
    * representation is expected to have been generated using {@link #toString()}. If the format
    * differs, this method will throw an {code IllegalArgumentException}.
    *
    * @param value the {@code String} representation of a {@link YoTuple3DDefinition}.
    * @return the parsed tuple 3D object.
    */
   public static YoTuple3DDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoTuple3D))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String x = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String y = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String z = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (x.equalsIgnoreCase("null"))
            x = null;
         if (y.equalsIgnoreCase("null"))
            y = null;
         if (z.equalsIgnoreCase("null"))
            z = null;
         if (frame.equalsIgnoreCase("null"))
            frame = null;

         return new YoTuple3DDefinition(x, y, z, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown tuple 3D format: " + value);
      }
   }
}
