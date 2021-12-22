package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoGraphicSTPBox3D")
public class YoGraphicSTPBox3DDefinition extends YoGraphicBox3DDefinition
{
   private String minimumMargin;
   private String maximumMargin;

   public void setMinimumMargin(double minimumMargin)
   {
      this.minimumMargin = Double.toString(minimumMargin);
   }

   @XmlElement
   public void setMinimumMargin(String minimumMargin)
   {
      this.minimumMargin = minimumMargin;
   }

   public void setMaximumMargin(double maximumMargin)
   {
      this.maximumMargin = Double.toString(maximumMargin);
   }

   @XmlElement
   public void setMaximumMargin(String maximumMargin)
   {
      this.maximumMargin = maximumMargin;
   }

   public String getMinimumMargin()
   {
      return minimumMargin;
   }

   public String getMaximumMargin()
   {
      return maximumMargin;
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
      else if (object instanceof YoGraphicSTPBox3DDefinition)
      {
         YoGraphicSTPBox3DDefinition other = (YoGraphicSTPBox3DDefinition) object;

         if (minimumMargin == null ? other.minimumMargin != null : !minimumMargin.equals(other.minimumMargin))
            return false;
         if (maximumMargin == null ? other.maximumMargin != null : !maximumMargin.equals(other.maximumMargin))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return super.toString() + ", minimumMargin: " + minimumMargin + ", maximumMargin: " + maximumMargin;
   }
}
