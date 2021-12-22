package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupModel;

public class YoCompositePattern
{
   private final String type;
   private final String[] componentIdentifiers;
   private final boolean crossRegistry;
   private final List<ChartGroupModel> preferredChartConfigurations;

   public static YoCompositePattern singleton(String type)
   {
      return new YoCompositePattern(type, false, null, new ArrayList<>());
   }

   public YoCompositePattern(String type, boolean crossRegistry, String[] componentIdentifiers, List<ChartGroupModel> preferredChartConfigurations)
   {
      this.type = type;
      this.crossRegistry = crossRegistry;

      if (componentIdentifiers != null)
      {
         componentIdentifiers = Stream.of(componentIdentifiers).toArray(String[]::new);

         if (preferredChartConfigurations == null)
            preferredChartConfigurations = new ArrayList<>();

         for (ChartGroupModel chartIdentifierList : preferredChartConfigurations)
         {
            if (chartIdentifierList.size() != componentIdentifiers.length)
            {
               throw new IllegalArgumentException("Unexpected chart configuration size for composite " + type + ".\n"
                     + "The size of each preferred chart configuration should be equal to the number of components in the composite.\n"
                     + "Problematic chart configuration: " + chartIdentifierList.getName() + " configuration size " + chartIdentifierList.size() + " expected "
                     + componentIdentifiers.length + ".");
            }
         }
      }

      this.componentIdentifiers = componentIdentifiers;
      this.preferredChartConfigurations = preferredChartConfigurations;
   }

   public String getType()
   {
      return type;
   }

   public String[] getComponentIdentifiers()
   {
      return componentIdentifiers;
   }

   public boolean isCrossRegistry()
   {
      return crossRegistry;
   }

   public List<ChartGroupModel> getPreferredChartConfigurations()
   {
      return preferredChartConfigurations;
   }

   @Override
   public String toString()
   {
      return type + ", components " + Arrays.toString(componentIdentifiers);
   }
}