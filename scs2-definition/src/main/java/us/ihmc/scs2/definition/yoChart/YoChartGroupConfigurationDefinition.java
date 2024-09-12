package us.ihmc.scs2.definition.yoChart;

import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoChartGroupConfiguration")
public class YoChartGroupConfigurationDefinition
{
   private String name;
   private int numberOfRows, numberOfColumns;
   private List<YoChartConfigurationDefinition> chartConfigurations;

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement
   public void setNumberOfRows(int numberOfRows)
   {
      this.numberOfRows = numberOfRows;
   }

   @XmlElement
   public void setNumberOfColumns(int numberOfColumns)
   {
      this.numberOfColumns = numberOfColumns;
   }

   @XmlElement
   public void setChartConfigurations(List<YoChartConfigurationDefinition> chartConfigurations)
   {
      this.chartConfigurations = chartConfigurations;
   }

   public String getName()
   {
      return name;
   }

   public int getNumberOfRows()
   {
      return numberOfRows;
   }

   public int getNumberOfColumns()
   {
      return numberOfColumns;
   }

   public List<YoChartConfigurationDefinition> getChartConfigurations()
   {
      return chartConfigurations;
   }
}
