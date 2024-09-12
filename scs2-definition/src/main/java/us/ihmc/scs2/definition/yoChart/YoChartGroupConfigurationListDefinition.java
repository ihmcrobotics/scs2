package us.ihmc.scs2.definition.yoChart;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@XmlRootElement(name = "YoChartGroupConfigurationList")
public class YoChartGroupConfigurationListDefinition
{
   private String name;
   private List<YoChartGroupConfigurationDefinition> chartGroupConfigurations;

   public YoChartGroupConfigurationListDefinition()
   {
   }

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement
   public void setChartGroupConfigurations(List<YoChartGroupConfigurationDefinition> chartGroupConfigurations)
   {
      this.chartGroupConfigurations = chartGroupConfigurations;
   }

   public void addChartGroupConfiguration(YoChartGroupConfigurationDefinition chartGroupConfiguration)
   {
      if (chartGroupConfigurations == null)
         chartGroupConfigurations = new ArrayList<>();
      chartGroupConfigurations.add(chartGroupConfiguration);
   }

   public String getName()
   {
      return name;
   }

   public List<YoChartGroupConfigurationDefinition> getChartGroupConfigurations()
   {
      return chartGroupConfigurations;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoChartGroupConfigurationListDefinition other)
      {
         if (!Objects.equals(name, other.name))
            return false;
         if (!Objects.equals(chartGroupConfigurations, other.chartGroupConfigurations))
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
      return "YoChartGroupConfigurationListDefinition [name=" + name + ", chartGroupConfigurations=" + chartGroupConfigurations + "]";
   }
}
