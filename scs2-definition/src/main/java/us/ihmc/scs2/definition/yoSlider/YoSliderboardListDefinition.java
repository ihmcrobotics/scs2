package us.ihmc.scs2.definition.yoSlider;

import us.ihmc.scs2.definition.configuration.WindowConfigurationDefinition;

import javax.xml.bind.annotation.XmlRootElement;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

@XmlRootElement(name = "YoSliderboardList")
public class YoSliderboardListDefinition
{
   private String name;
   private List<YoSliderboardDefinition> yoSliderboards = new ArrayList<>();

   // TODO This doesn't really belong here. Need to fix the whole configuration file format.
   private WindowConfigurationDefinition windowConfiguration;

   public YoSliderboardListDefinition()
   {
   }

   public YoSliderboardListDefinition(String name, YoSliderboardDefinition yoSliderboard)
   {
      this(null, Collections.singletonList(yoSliderboard));
   }

   public YoSliderboardListDefinition(String name, List<YoSliderboardDefinition> yoSliderboards)
   {
      this.name = name;
      this.yoSliderboards = yoSliderboards;
   }

   public YoSliderboardListDefinition(YoSliderboardListDefinition other)
   {
      set(other);
   }

   public void set(YoSliderboardListDefinition other)
   {
      name = other.name;

      if (other.yoSliderboards != null)
         yoSliderboards = other.yoSliderboards.stream().map(YoSliderboardDefinition::new).collect(Collectors.toList());
      else
         yoSliderboards = null;

      if (other.windowConfiguration != null)
         windowConfiguration = new WindowConfigurationDefinition(other.windowConfiguration);
      else
         windowConfiguration = null;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setYoSliderboards(List<YoSliderboardDefinition> yoSliderboards)
   {
      this.yoSliderboards = yoSliderboards;
   }

   public void setWindowConfiguration(WindowConfigurationDefinition windowConfiguration)
   {
      this.windowConfiguration = windowConfiguration;
   }

   public String getName()
   {
      return name;
   }

   public List<YoSliderboardDefinition> getYoSliderboards()
   {
      return yoSliderboards;
   }

   public WindowConfigurationDefinition getWindowConfiguration()
   {
      return windowConfiguration;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoSliderboardListDefinition other)
      {
         if (!Objects.equals(name, other.name))
            return false;
         if (!Objects.equals(yoSliderboards, other.yoSliderboards))
            return false;
         if (!Objects.equals(windowConfiguration, other.windowConfiguration))
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
      return "name:" + name + ", yoSliderboards:" + yoSliderboards + ", windowConfiguration:" + windowConfiguration;
   }
}
