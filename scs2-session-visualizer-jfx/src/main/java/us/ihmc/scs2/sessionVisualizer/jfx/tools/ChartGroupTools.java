package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupLayout;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupModel;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIdentifier;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.yoVariables.variable.YoVariable;

public class ChartGroupTools
{
   public static List<ChartGroupLayout> toChartGroupLayouts(List<YoComposite> yoComposites)
   {
      if (yoComposites == null || yoComposites.isEmpty())
         return null;

      YoComposite firstComposite = yoComposites.get(0);

      if (yoComposites.size() == 1)
         return toChartGroupLayouts(firstComposite);

      boolean areAllSameType = yoComposites.subList(1, yoComposites.size()).stream()
                                           .allMatch(composite -> composite.getClass().equals(firstComposite.getClass()));

      if (areAllSameType && !firstComposite.getPattern().getPreferredChartConfigurations().isEmpty() && firstComposite.getYoComponents().size() > 1)
      {
         List<ChartGroupLayout> chartGroupLayouts = new ArrayList<>();
         List<ChartGroupModel> models = firstComposite.getPattern().getPreferredChartConfigurations();

         for (int i = 0; i < models.size(); i++)
         {
            ChartGroupModel model = models.get(i);
            Map<ChartIdentifier, List<? extends YoVariable>> layout = new LinkedHashMap<>();

            List<ChartIdentifier> chartIdentifiers = model.getChartIdentifiers();

            for (int j = 0; j < chartIdentifiers.size(); j++)
            {
               int jFinal = j;
               layout.put(chartIdentifiers.get(j),
                          yoComposites.stream().map(yoComposite -> yoComposite.getYoComponents().get(jFinal)).collect(Collectors.toList()));
            }

            chartGroupLayouts.add(new ChartGroupLayout(model.getName(), layout));
         }
         return chartGroupLayouts;
      }
      else
      {
         return Arrays.asList(singleChartLayout(yoComposites), horizontalLayout(yoComposites), verticalLayout(yoComposites));
      }
   }

   public static List<ChartGroupLayout> toChartGroupLayouts(YoComposite yoComposite)
   {
      return yoComposite.getPattern().getPreferredChartConfigurations().stream().map(model -> new ChartGroupLayout(model, yoComposite))
                        .collect(Collectors.toList());
   }

   public static ChartGroupLayout singleChartLayout(List<YoComposite> yoComposites)
   {
      if (yoComposites == null || yoComposites.isEmpty())
         return null;

      List<YoVariable> yoVariables = yoComposites.stream().flatMap(yoComposite -> yoComposite.getYoComponents().stream()).collect(Collectors.toList());

      return new ChartGroupLayout("Single", Collections.singletonMap(new ChartIdentifier(0, 0), yoVariables));
   }

   public static ChartGroupLayout horizontalLayout(List<YoComposite> yoComposites)
   {
      if (yoComposites == null || yoComposites.isEmpty())
         return null;

      Map<ChartIdentifier, List<? extends YoVariable>> layout = new LinkedHashMap<>();
      for (int i = 0; i < yoComposites.size(); i++)
         layout.put(new ChartIdentifier(0, i), yoComposites.get(i).getYoComponents());
      return new ChartGroupLayout("Horizontal", layout);
   }

   public static ChartGroupLayout verticalLayout(List<YoComposite> yoComposites)
   {
      if (yoComposites == null || yoComposites.isEmpty())
         return null;

      Map<ChartIdentifier, List<? extends YoVariable>> layout = new LinkedHashMap<>();
      for (int i = 0; i < yoComposites.size(); i++)
         layout.put(new ChartIdentifier(i, 0), yoComposites.get(i).getYoComponents());
      return new ChartGroupLayout("Vertical", layout);
   }
}
