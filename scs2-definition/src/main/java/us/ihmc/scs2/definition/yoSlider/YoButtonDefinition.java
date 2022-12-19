package us.ihmc.scs2.definition.yoSlider;

import java.util.Objects;

import javax.xml.bind.annotation.XmlAttribute;

public class YoButtonDefinition
{

   private String variableName;
   private int index = -1;

   public YoButtonDefinition()
   {
   }

   public YoButtonDefinition(String variableName, int index)
   {
      this.variableName = variableName;
      this.index = index;
   }

   @XmlAttribute
   public void setVariableName(String variableName)
   {
      this.variableName = variableName;
   }

   @XmlAttribute
   public void setIndex(int index)
   {
      this.index = index;
   }

   public String getVariableName()
   {
      return variableName;
   }

   public int getIndex()
   {
      return index;
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

         if (!Objects.equals(variableName, other.variableName))
            return false;
         if (index != other.index)
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
      return "variableName:" + variableName + ", index:" + index;
   }
}
