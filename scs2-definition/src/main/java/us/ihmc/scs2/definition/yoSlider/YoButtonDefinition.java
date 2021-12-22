package us.ihmc.scs2.definition.yoSlider;

import javax.xml.bind.annotation.XmlAttribute;

public class YoButtonDefinition
{
   private String variableName;

   @XmlAttribute
   public void setVariableName(String variableName)
   {
      this.variableName = variableName;
   }

   public String getVariableName()
   {
      return variableName;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoButtonDefinition)
      {
         YoButtonDefinition other = (YoButtonDefinition) object;

         if (variableName == null ? other.variableName != null : variableName.equals(other.variableName))
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
      return "variableName:" + variableName;
   }
}
