package us.ihmc.scs2.definition.yoGraphic;

import java.util.List;

import javax.xml.bind.annotation.XmlElement;

public class YoGraphicPolynomial3DDefinition extends YoGraphic3DDefinition
{
   private List<String> coefficientsX, coefficientsY, coefficientsZ;
   private String numberOfCoefficientsX, numberOfCoefficientsY, numberOfCoefficientsZ;
   private String startTime, endTime;
   private String size;

   @XmlElement(name = "coefficientX")
   public void setCoefficientsX(List<String> coefficientsX)
   {
      this.coefficientsX = coefficientsX;
   }

   @XmlElement(name = "coefficientY")
   public void setCoefficientsY(List<String> coefficientsY)
   {
      this.coefficientsY = coefficientsY;
   }

   @XmlElement(name = "coefficientZ")
   public void setCoefficientsZ(List<String> coefficientsZ)
   {
      this.coefficientsZ = coefficientsZ;
   }

   @XmlElement
   public void setNumberOfCoefficientsX(String numberOfCoefficientsX)
   {
      this.numberOfCoefficientsX = numberOfCoefficientsX;
   }

   @XmlElement
   public void setNumberOfCoefficientsY(String numberOfCoefficientsY)
   {
      this.numberOfCoefficientsY = numberOfCoefficientsY;
   }

   @XmlElement
   public void setNumberOfCoefficientsZ(String numberOfCoefficientsZ)
   {
      this.numberOfCoefficientsZ = numberOfCoefficientsZ;
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

   public List<String> getCoefficientsX()
   {
      return coefficientsX;
   }

   public List<String> getCoefficientsY()
   {
      return coefficientsY;
   }

   public List<String> getCoefficientsZ()
   {
      return coefficientsZ;
   }

   public String getNumberOfCoefficientsX()
   {
      return numberOfCoefficientsX;
   }

   public String getNumberOfCoefficientsY()
   {
      return numberOfCoefficientsY;
   }

   public String getNumberOfCoefficientsZ()
   {
      return numberOfCoefficientsZ;
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

         if (coefficientsX == null ? other.coefficientsX != null : !coefficientsX.equals(other.coefficientsX))
            return false;
         if (coefficientsY == null ? other.coefficientsY != null : !coefficientsY.equals(other.coefficientsY))
            return false;
         if (coefficientsZ == null ? other.coefficientsZ != null : !coefficientsZ.equals(other.coefficientsZ))
            return false;
         if (numberOfCoefficientsX == null ? other.numberOfCoefficientsX != null : !numberOfCoefficientsX.equals(other.numberOfCoefficientsX))
            return false;
         if (numberOfCoefficientsY == null ? other.numberOfCoefficientsY != null : !numberOfCoefficientsY.equals(other.numberOfCoefficientsY))
            return false;
         if (numberOfCoefficientsZ == null ? other.numberOfCoefficientsZ != null : !numberOfCoefficientsZ.equals(other.numberOfCoefficientsZ))
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
