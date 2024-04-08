package us.ihmc.scs2.definition.yoGraphic;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import javax.xml.bind.annotation.XmlElement;
import java.util.Objects;

/**
 * A {@code YoGraphicPolynomial3DDefinition} is a template for creating 3D polynomial and which
 * components can be backed by {@code YoVariable}s. <br>
 * <img src=
 * "https://github.com/ihmcrobotics/simulation-construction-set-2/wiki/images/YoGraphicJavadoc/YoPolynomialFX3D.png"
 * width=150px/>
 * <p>
 * The {@code YoGraphicPolynomial3DDefinition} is to be passed before initialization of a session
 * (either before starting a simulation or when creating a yoVariable server), such that the SCS GUI
 * can use the definitions and create the actual graphics.
 * </p>
 * <p>
 * See {@link YoGraphicDefinitionFactory} for factory methods simplifying the creation of yoGraphic
 * definitions.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class YoGraphicPolynomial3DDefinition extends YoGraphic3DDefinition
{
   /**
    * The list of the polynomial coefficients for the x-axis. The coefficients are ordered from low to
    * high order.
    */
   private YoListDefinition coefficientsX;
   /**
    * The list of the polynomial coefficients for the y-axis. The coefficients are ordered from low to
    * high order.
    */
   private YoListDefinition coefficientsY;
   /**
    * The list of the polynomial coefficients for the z-axis. The coefficients are ordered from low to
    * high order.
    */
   private YoListDefinition coefficientsZ;
   /** The reference frame in which the polynomial is expressed. */
   private String referenceFrame;

   /** The polynomial is rendered in the time domain <tt>t&in;[startTime, endTime]</tt> */
   private String startTime, endTime;

   /** The radius the graphic. */
   private String size;
   /** The number of divisions in the time domain. */
   private String timeResolution;
   /** The number of radial divisions. */
   private String numberOfDivisions;

   /**
    * Creates a new yoGraphic definition for rendering a polynomial.
    * <p>
    * Its components need to be initialized. See {@link YoGraphicDefinitionFactory} for factories to
    * facilitate creation.
    * </p>
    */
   public YoGraphicPolynomial3DDefinition()
   {
   }

   /**
    * Copy constructor.
    *
    * @param other the other definition to copy. Not modified.
    */
   public YoGraphicPolynomial3DDefinition(YoGraphicPolynomial3DDefinition other)
   {
      super(other);
      coefficientsX = other.coefficientsX == null ? null : other.coefficientsX.copy();
      coefficientsY = other.coefficientsY == null ? null : other.coefficientsY.copy();
      coefficientsZ = other.coefficientsZ == null ? null : other.coefficientsZ.copy();
      referenceFrame = other.referenceFrame;
      startTime = other.startTime;
      endTime = other.endTime;
      size = other.size;
      timeResolution = other.timeResolution;
      numberOfDivisions = other.numberOfDivisions;
   }

   @Override
   protected void registerFields()
   {
      super.registerFields();
      registerYoListField("coefficientsX", this::getCoefficientsX, this::setCoefficientsX);
      registerYoListField("coefficientsY", this::getCoefficientsY, this::setCoefficientsY);
      registerYoListField("coefficientsZ", this::getCoefficientsZ, this::setCoefficientsZ);
      registerStringField("referenceFrame", this::getReferenceFrame, this::setReferenceFrame);
      registerStringField("startTime", this::getStartTime, this::setStartTime);
      registerStringField("endTime", this::getEndTime, this::setEndTime);
      registerStringField("size", this::getSize, this::setSize);
      registerStringField("timeResolution", this::getTimeResolution, this::setTimeResolution);
      registerStringField("numberOfDivisions", this::getNumberOfDivisions, this::setNumberOfDivisions);
   }

   /**
    * Sets the coefficients for the polynomial on the x-axis. The coefficients are ordered from low to
    * high order.
    *
    * @param coefficientsX the x-axis polynomial's coefficients.
    */
   @XmlElement(name = "coefficientsX")
   public void setCoefficientsX(YoListDefinition coefficientsX)
   {
      this.coefficientsX = coefficientsX;
   }

   /**
    * Sets the coefficients for the polynomial on the y-axis. The coefficients are ordered from low to
    * high order.
    *
    * @param coefficientsY the y-axis polynomial's coefficients.
    */
   @XmlElement(name = "coefficientsY")
   public void setCoefficientsY(YoListDefinition coefficientsY)
   {
      this.coefficientsY = coefficientsY;
   }

   /**
    * Sets the coefficients for the polynomial on the z-axis. The coefficients are ordered from low to
    * high order.
    *
    * @param coefficientsZ the z-axis polynomial's coefficients.
    */
   @XmlElement(name = "coefficientsZ")
   public void setCoefficientsZ(YoListDefinition coefficientsZ)
   {
      this.coefficientsZ = coefficientsZ;
   }

   /**
    * Sets the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in which the
    * polynomial is to be expressed, or {@code null} if it is expressed in world frame.
    *
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()} of the reference frame.
    */
   @XmlElement(name = "referenceFrame")
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /**
    * Sets the initial time for the polynomial. The polynomial is visualize in the time domain
    * <tt>t&in;[startTime, endTime]</tt>.
    * <p>
    * Using this method sets it to a constant value.
    * </p>
    *
    * @param startTime the initial time.
    */
   public void setStartTime(double startTime)
   {
      setStartTime(Double.toString(startTime));
   }

   /**
    * Sets the initial time for the polynomial. The polynomial is visualize in the time domain
    * <tt>t&in;[startTime, endTime]</tt>.
    * <p>
    * Using this method allows to back the start time with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    *
    * @param startTime the initial time.
    */
   @XmlElement
   public void setStartTime(String startTime)
   {
      this.startTime = startTime;
   }

   /**
    * Sets the final time for the polynomial. The polynomial is visualize in the time domain
    * <tt>t&in;[startTime, endTime]</tt>.
    * <p>
    * Using this method sets it to a constant value.
    * </p>
    *
    * @param endTime the final time.
    */
   public void setEndTime(double endTime)
   {
      setEndTime(Double.toString(endTime));
   }

   /**
    * Sets the final time for the polynomial. The polynomial is visualize in the time domain
    * <tt>t&in;[endTime, endTime]</tt>.
    * <p>
    * Using this method allows to back the end time with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    *
    * @param endTime the final time.
    */
   @XmlElement
   public void setEndTime(String endTime)
   {
      this.endTime = endTime;
   }

   /**
    * Sets the size of the graphic.
    * <p>
    * Using this method sets it to a constant value.
    * </p>
    *
    * @param size the size of the graphic.
    */
   public void setSize(double size)
   {
      setSize(Double.toString(size));
   }

   /**
    * Sets the size of the graphic.
    * <p>
    * Using this method allows to back the size with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    *
    * @param size the size of the graphic.
    */
   @XmlElement
   public void setSize(String size)
   {
      this.size = size;
   }

   /**
    * Sets the number of divisions for the time domain to a constant value.
    * <p>
    * Using this method sets it to a constant value.
    * </p>
    *
    * @param timeResolution the time resolution.
    */
   public void setTimeResolution(int timeResolution)
   {
      setTimeResolution(Integer.toString(timeResolution));
   }

   /**
    * Sets the number of divisions for the time domain to a constant value.
    * <p>
    * Using this method allows to back the resoluation with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    *
    * @param timeResolution the time resolution.
    */
   @XmlElement
   public void setTimeResolution(String timeResolution)
   {
      this.timeResolution = timeResolution;
   }

   /**
    * Sets the number of radial divisions.
    * <p>
    * Using this method sets it to a constant value.
    * </p>
    *
    * @param numberOfDivisions the number of radial divisions.
    */
   public void setNumberOfDivisions(int numberOfDivisions)
   {
      setNumberOfDivisions(Integer.toString(numberOfDivisions));
   }

   /**
    * Sets the number of radial divisions.
    * <p>
    * Using this method allows to back the resoluation with a {@code YoVariable} by giving the variable
    * name/fullname.
    * </p>
    *
    * @param numberOfDivisions the number of radial divisions.
    */
   @XmlElement
   public void setNumberOfDivisions(String numberOfDivisions)
   {
      this.numberOfDivisions = numberOfDivisions;
   }

   public YoListDefinition getCoefficientsX()
   {
      return coefficientsX;
   }

   public YoListDefinition getCoefficientsY()
   {
      return coefficientsY;
   }

   public YoListDefinition getCoefficientsZ()
   {
      return coefficientsZ;
   }

   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   public String getStartTime()
   {
      return startTime;
   }

   public String getEndTime()
   {
      return endTime;
   }

   public String getSize()
   {
      return size;
   }

   public String getTimeResolution()
   {
      return timeResolution;
   }

   public String getNumberOfDivisions()
   {
      return numberOfDivisions;
   }

   @Override
   public YoGraphicPolynomial3DDefinition copy()
   {
      return new YoGraphicPolynomial3DDefinition(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (!super.equals(object))
      {
         return false;
      }
      else if (object instanceof YoGraphicPolynomial3DDefinition other)
      {
         if (!Objects.equals(coefficientsX, other.coefficientsX))
            return false;
         if (!Objects.equals(coefficientsY, other.coefficientsY))
            return false;
         if (!Objects.equals(coefficientsZ, other.coefficientsZ))
            return false;
         if (!Objects.equals(referenceFrame, other.referenceFrame))
            return false;
         if (!Objects.equals(startTime, other.startTime))
            return false;
         if (!Objects.equals(endTime, other.endTime))
            return false;
         if (!Objects.equals(size, other.size))
            return false;
         if (!Objects.equals(timeResolution, other.timeResolution))
            return false;
         if (!Objects.equals(numberOfDivisions, other.numberOfDivisions))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
