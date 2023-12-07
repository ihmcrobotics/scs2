package us.ihmc.scs2.definition.yoChart;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.ArrayList;
import java.util.List;

@XmlRootElement(name = "YoChartGroupConfigurationList")
public class YoChartGroupConfigurationListDefinition
{
   private List<YoChartGroupConfigurationDefinition> chartGroupConfigurations;

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

   public List<YoChartGroupConfigurationDefinition> getChartGroupConfigurations()
   {
      return chartGroupConfigurations;
   }
}
