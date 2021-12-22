package us.ihmc.scs2.definition.yoSlider;

import java.util.List;

import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoSliderboardList")
public class YoSliderboardListDefinition
{
   private String name;
   private List<YoSliderboardDefinition> yoSliderboards;

   public YoSliderboardListDefinition()
   {
   }

   public YoSliderboardListDefinition(String name, List<YoSliderboardDefinition> yoSliderboards)
   {
      this.name = name;
      this.yoSliderboards = yoSliderboards;
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
