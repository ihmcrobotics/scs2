package us.ihmc.scs2.definition.yoChart;

import java.util.List;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "ChartConfiguration")
public class YoChartConfigurationDefinition
{
   private YoChartIdentifierDefinition identifier;
   private String chartStyle;
   private List<String> yoVariables;
   private List<ChartDoubleBoundsDefinition> yBounds;
   private List<Boolean> negates;

   @XmlElement
   public void setIdentifier(YoChartIdentifierDefinition identifier)
   {
      this.identifier = identifier;
   }

   @XmlElement
   public void setChartStyle(String chartStyle)
   {
      this.chartStyle = chartStyle;
   }

   @XmlElement
   public void setYoVariables(List<String> yoVariables)
   {
      this.yoVariables = yoVariables;
   }

   @XmlElement
   public void setYBounds(List<ChartDoubleBoundsDefinition> yBounds)
   {
      this.yBounds = yBounds;
   }

   @XmlElement
   public void setNegates(List<Boolean> negates)
   {
      this.negates = negates;
   }

   public YoChartIdentifierDefinition getIdentifier()
   {
      return identifier;
   }

   public String getChartStyle()
   {
      return chartStyle;
   }

   public List<String> getYoVariables()
   {
      return yoVariables;
   }

   public List<ChartDoubleBoundsDefinition> getYBounds()
   {
      return yBounds;
   }

   public List<Boolean> getNegates()
   {
      return negates;
   }
}
