package us.ihmc.scs2.definition.yoSlider;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoSliderboardList")
public class YoSliderboardListDefinition
{
   private String name;
   private List<YoSliderboardDefinition> yoSliderboards = new ArrayList<>();

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
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setYoSliderboards(List<YoSliderboardDefinition> yoSliderboards)
   {
      this.yoSliderboards = yoSliderboards;
   }

   public String getName()
   {
      return name;
   }

   public List<YoSliderboardDefinition> getYoSliderboards()
   {
      return yoSliderboards;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoSliderboardListDefinition)
      {
         YoSliderboardListDefinition other = (YoSliderboardListDefinition) object;

         if (name == null ? other.name != null : !name.equals(other.name))
            return false;
         if (yoSliderboards == null ? other.yoSliderboards != null : !yoSliderboards.equals(other.yoSliderboards))
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
      return "name:" + name + ", yoSliderboards:" + yoSliderboards;
   }
}
