package us.ihmc.scs2.definition.yoChart;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.scs2.definition.yoEntry.YoEntryDefinition;
import us.ihmc.scs2.definition.yoEntry.YoEntryListDefinition;

@XmlRootElement(name = "ChartConfiguration")
public class YoChartConfigurationDefinition
{
   private YoChartIdentifierDefinition identifier;
   private String chartStyle;
   private List<String> yoVariables;
   private List<ChartDoubleBoundsDefinition> yBounds;
   private List<Boolean> negates;

   public YoChartConfigurationDefinition()
   {
   }

   public YoChartConfigurationDefinition(Collection<String> variables)
   {
      this.yoVariables = new ArrayList<>(variables);
   }

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

   public static Pair<String, YoChartConfigurationDefinition> newYoVariableChartList(String name, Collection<String> variableNames)
   {
      return new ImmutablePair<>(name, new YoChartConfigurationDefinition(variableNames));
   }
}
