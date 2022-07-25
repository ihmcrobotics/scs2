package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

public class YoGraphicPolynomial3DDefinition extends YoGraphic3DDefinition
{
   private YoListDefinition coefficientsX;
   private YoListDefinition coefficientsY;
   private YoListDefinition coefficientsZ;

   private String startTime, endTime;
   private String size;

   @XmlElement(name = "coefficientsX")
   public void setCoefficientsX(YoListDefinition coefficientsX)
   {
      this.coefficientsX = coefficientsX;
   }

   @XmlElement(name = "coefficientsY")
   public void setCoefficientsY(YoListDefinition coefficientsY)
   {
      this.coefficientsY = coefficientsY;
   }

   @XmlElement(name = "coefficientsZ")
   public void setCoefficientsZ(YoListDefinition coefficientsZ)
   {
      this.coefficientsZ = coefficientsZ;
   }

   public void setStartTime(double startTime)
   {
      setStartTime(Double.toString(startTime));
   }

   @XmlElement
   public void setStartTime(String startTime)
   {
      this.startTime = startTime;
   }

   public void setEndTime(double endTime)
   {
      setEndTime(Double.toString(endTime));
   }

   @XmlElement
   public void setEndTime(String endTime)
   {
      this.endTime = endTime;
   }

   public void setSize(double size)
   {
      setSize(Double.toString(size));
   }

   @XmlElement
   public void setSize(String size)
   {
      this.size = size;
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
      else if (object instanceof YoGraphicPolynomial3DDefinition)
      {
         YoGraphicPolynomial3DDefinition other = (YoGraphicPolynomial3DDefinition) object;

         if (!Objects.equals(coefficientsX, other.coefficientsX))
            return false;
         if (!Objects.equals(coefficientsY, other.coefficientsY))
            return false;
         if (!Objects.equals(coefficientsZ, other.coefficientsZ))
            return false;
         if (startTime == null ? other.startTime != null : !startTime.equals(other.startTime))
            return false;
         if (endTime == null ? other.endTime != null : !endTime.equals(other.endTime))
            return false;
         if (size == null ? other.size != null : !size.equals(other.size))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
