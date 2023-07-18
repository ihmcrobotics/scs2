package us.ihmc.scs2.definition.yoSlider;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

/**
 * Definition of a sliderboard.
 * 
 * @author Sylvain Bertrand
 */
public class YoSliderboardDefinition
{
   /**
    * Name of this sliderboard definition to facilitate retrieval and can be used to display in GUIs.
    */
   private String name;

   /**
    * Type value for the BCF2000.
    */
   public static final String BCF2000 = "BCF2000";
   public static final String XTOUCHCOMPACT = "XTOUCHCompact";

   /**
    * The type of sliderboard to use with this definition.
    */
   private String type = BCF2000;
   private List<YoKnobDefinition> knobs = new ArrayList<>();
   private List<YoButtonDefinition> buttons = new ArrayList<>();
   private List<YoSliderDefinition> sliders = new ArrayList<>();

   public YoSliderboardDefinition()
   {
   }

   public YoSliderboardDefinition(String name)
   {
      this.name = name;
   }

   public YoSliderboardDefinition(String name, String type)
   {
      this.name = name;
      this.type = type;
   }

   public YoSliderboardDefinition(YoSliderboardDefinition other)
   {
      set(other);
   }

   public void set(YoSliderboardDefinition other)
   {
      name = other.name;
      type = other.type;
      if (other.knobs != null)
         knobs = other.knobs.stream().map(YoKnobDefinition::new).collect(Collectors.toList());
      else
         knobs = null;
      if (other.buttons != null)
         buttons = other.buttons.stream().map(YoButtonDefinition::new).collect(Collectors.toList());
      else
         buttons = null;
      if (other.sliders != null)
         sliders = other.sliders.stream().map(YoSliderDefinition::new).collect(Collectors.toList());
      else
         sliders = null;
   }

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
