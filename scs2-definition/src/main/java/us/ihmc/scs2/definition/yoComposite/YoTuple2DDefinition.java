package us.ihmc.scs2.definition.yoComposite;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

/**
 * A {@link YoTuple2DDefinition} represents a template for creating a tuple 2D which components can
 * be backed by {@code YoVariable}s.
 * <p>
 * A tuple 2D can be used to represent either a point or a vector.
 * </p>
 * <p>
 * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
 * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
 * to a constant value by using for instance {@link Double#toString(double)}.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods to facilitate the creation of a
 * {@code YoTuple2DDefinition}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlRootElement(name = "YoTuple2D")
public class YoTuple2DDefinition extends YoCompositeDefinition
{
   public static final String YoTuple2D = "YoTuple2D";
   public static final String[] YoTuple2DIdentifiers = new String[] {"x", "y"};

   /**
    * The components of this tuple 2D.
    * <p>
    * Each component can be backed by a {@code YoVariable} by setting it to the variable name or
    * fullname. Note that using the fullname is preferable to avoid name collisions. It can also be set
    * to a constant value by using for instance {@link Double#toString(double)}.
    * </p>
    */
   private String x, y;
   /**
    * The name id ({@link ReferenceFrame#getNameId()}) of the reference frame this tuple 2D is
    * expressed in, or {@code null} if it is expressed in world frame.
    */
   private String referenceFrame;

   /** Creates an empty tuple 2D which components need to be initialized. */
   public YoTuple2DDefinition()
   {
   }

   /**
    * Creates a new tuple 2D that is expressed in world frame.
    *
    * @param x the constant value representation or {@code YoVariable} name/fullname for the
    *          x-component.
    * @param y the constant value representation or {@code YoVariable} name/fullname for the
    *          y-component.
    */
   public YoTuple2DDefinition(String x, String y)
   {
      this(x, y, null);
   }

   /**
    * Creates a new tuple 2D that is expressed in a specific frame.
    *
    * @param x              the constant value representation or {@code YoVariable} name/fullname for
    *                       the x-component.
    * @param y              the constant value representation or {@code YoVariable} name/fullname for
    *                       the y-component.
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in
    *                       which this tuple 2D is to be expressed. Note that not all reference frames
    *                       are available from inside SCS2.
    */
   public YoTuple2DDefinition(String x, String y, String referenceFrame)
   {
      this.x = x;
      this.y = y;
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

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoTuple2D;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoTuple2DIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {x, y};
   }

   /**
    * Creates a copy of this tuple 2D definition.
    *
    * @return the copy.
    */
   public YoTuple2DDefinition copy()
   {
      return new YoTuple2DDefinition(x, y, referenceFrame);
   }

   /**
    * Parses the given {@code value} into a {@link YoTuple2DDefinition}. The given {@code String}
    * representation is expected to have been generated using {@link #toString()}. If the format
    * differs, this method will throw an {code IllegalArgumentException}.
    *
    * @param value the {@code String} representation of a {@link YoTuple2DDefinition}.
    * @return the parsed tuple 2D object.
    */
   public static YoTuple2DDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoTuple2D))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String x = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String y = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (x.equalsIgnoreCase("null"))
            x = null;
         if (y.equalsIgnoreCase("null"))
            y = null;
         if (frame.equalsIgnoreCase("null"))
            frame = null;

         return new YoTuple2DDefinition(x, y, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown tuple 2D format: " + value);
      }
   }
}
