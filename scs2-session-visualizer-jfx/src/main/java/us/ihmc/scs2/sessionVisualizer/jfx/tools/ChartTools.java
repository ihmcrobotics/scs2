package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import us.ihmc.scs2.definition.yoChart.ChartDoubleBoundsDefinition;
import us.ihmc.scs2.definition.yoChart.YoChartGroupModelDefinition;
import us.ihmc.scs2.definition.yoChart.YoChartIdentifierDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartDoubleBounds;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupModel;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIdentifier;
import us.ihmc.yoVariables.variable.*;

import java.util.List;
import java.util.stream.Collectors;

public class ChartTools
{
   public static ChartIdentifier toChartIdentifier(YoChartIdentifierDefinition xmlChartIdentifier)
   {
      return new ChartIdentifier(xmlChartIdentifier.getRow(), xmlChartIdentifier.getColumn());
   }

   public static ChartGroupModel toChartIdentifierList(YoChartGroupModelDefinition definition)
   {
      return new ChartGroupModel(definition.getName(),
                                 definition.getChartIdentifiers().stream().map(ChartTools::toChartIdentifier).collect(Collectors.toList()));
   }

   public static ChartDoubleBounds toChartDoubleBounds(ChartDoubleBoundsDefinition definition)
   {
      if (Double.isNaN(definition.getLower()) && Double.isNaN(definition.getUpper()))
         return null;
      else
         return new ChartDoubleBounds(definition.getLower(), definition.getUpper());
   }

   public static List<YoChartGroupModelDefinition> toYoChartGroupModelDefinitions(List<ChartGroupModel> chartGroupModels)
   {
      return chartGroupModels.stream().map(ChartTools::toYoChartGroupModelDefinition).collect(Collectors.toList());
   }

   public static YoChartGroupModelDefinition toYoChartGroupModelDefinition(ChartGroupModel chartGroupModel)
   {
      YoChartGroupModelDefinition definition = new YoChartGroupModelDefinition();
      definition.setName(chartGroupModel.getName());
      definition.setChartIdentifiers(toYoChartIdentifierDefinitions(chartGroupModel.getChartIdentifiers()));
      return definition;
   }

   public static List<YoChartIdentifierDefinition> toYoChartIdentifierDefinitions(List<ChartIdentifier> chartIdentifiers)
   {
      return chartIdentifiers.stream().map(ChartTools::toYoChartIdentifierDefinition).collect(Collectors.toList());
   }

   public static YoChartIdentifierDefinition toYoChartIdentifierDefinition(ChartIdentifier chartIdentifier)
   {
      if (chartIdentifier == null)
         return null;
      return new YoChartIdentifierDefinition(chartIdentifier.getRow(), chartIdentifier.getColumn());
   }

   public static ChartDoubleBoundsDefinition toChartDoubleBoundsDefinition(ChartDoubleBounds bounds)
   {
      if (bounds == null)
         return new ChartDoubleBoundsDefinition(Double.NaN, Double.NaN);
      else
         return new ChartDoubleBoundsDefinition(bounds.getLower(), bounds.getUpper());
   }

   public static String defaultYoVariableValueFormatter(YoVariable yoVariable, int precision)
   {
      if (yoVariable instanceof YoDouble yoDouble)
         return NumberFormatTools.doubleToString(yoDouble.getValue(), precision);
      else if (yoVariable instanceof YoBoolean yoBoolean)
         return Boolean.toString(yoBoolean.getValue());
      if (yoVariable instanceof YoInteger yoInteger)
         return Integer.toString(yoInteger.getValue());
      if (yoVariable instanceof YoLong yoLong)
         return Long.toString(yoLong.getValue());
      if (yoVariable instanceof YoEnum<?> yoEnum)
         return yoEnum.getStringValue();

      throw new UnsupportedOperationException("Unsupported YoVariable type: " + yoVariable.getClass().getSimpleName());
   }
}