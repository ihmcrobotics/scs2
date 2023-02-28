package us.ihmc.scs2.definition.yoGraphic;

import java.util.Objects;

import javax.xml.bind.annotation.XmlElement;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;

public abstract class YoGraphic3DDefinition extends YoGraphicDefinition
{
   protected PaintDefinition color;

   // For backward compatibility.
   @Deprecated
   @XmlElement(name = "color")
   public void setColorOld(ColorDefinition color)
   {
      LogTools.info("Setting color {}", color);
      this.color = color;
   }

   // For backward compatibility.
   @Deprecated
   public ColorDefinition getColorOld()
   {
      return null;
   }

   @XmlElement(name = "colorNew")
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
      else if (object instanceof YoGraphic3DDefinition other)
      {
         if (!Objects.equals(color, other.color))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
