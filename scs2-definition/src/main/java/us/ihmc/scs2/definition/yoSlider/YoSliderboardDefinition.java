package us.ihmc.scs2.definition.yoSlider;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class YoSliderboardDefinition
{
   private String name;
   private String type;
   private List<YoKnobDefinition> knobs;
   private List<YoButtonDefinition> buttons;
   private List<YoSliderDefinition> sliders;

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlAttribute
   public void setType(String type)
   {
      this.type = type;
   }

   @XmlElement
   public void setKnobs(List<YoKnobDefinition> knobs)
   {
      this.knobs = knobs;
   }

   @XmlElement
   public void setButtons(List<YoButtonDefinition> buttons)
   {
      this.buttons = buttons;
   }

   @XmlElement
   public void setSliders(List<YoSliderDefinition> sliders)
   {
      this.sliders = sliders;
   }

   public String getName()
   {
      return name;
   }

   public String getType()
   {
      return type;
   }

   public List<YoKnobDefinition> getKnobs()
   {
      return knobs;
   }

   public List<YoButtonDefinition> getButtons()
   {
      return buttons;
   }

   public List<YoSliderDefinition> getSliders()
   {
      return sliders;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoSliderboardDefinition)
      {
         YoSliderboardDefinition other = (YoSliderboardDefinition) object;

         if (name == null ? other.name != null : name.equals(other.name))
            return false;
         if (knobs == null ? other.knobs != null : knobs.equals(other.knobs))
            return false;
         if (buttons == null ? other.buttons != null : buttons.equals(other.buttons))
            return false;
         if (sliders == null ? other.sliders != null : sliders.equals(other.sliders))
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
      return "name: " + name + ", type: " + type + ", knobs: " + knobs + ", buttons: " + buttons + ", sliders: " + sliders;
   }
}
