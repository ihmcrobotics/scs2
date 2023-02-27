package us.ihmc.scs2.definition.yoGraphic;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.scs2.definition.visual.PaintDefinition;

public abstract class YoGraphic3DDefinition extends YoGraphicDefinition
{
   protected PaintDefinition color;

   @XmlElement
   public final void setColor(PaintDefinition color)
   {
      this.color = color;
   }

   public final PaintDefinition getColor()
   {
      return color;
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
      else if (object instanceof YoGraphic3DDefinition)
      {
         YoGraphic3DDefinition other = (YoGraphic3DDefinition) object;
         
         if (color == null ? other.color != null : !color.equals(other.color))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
