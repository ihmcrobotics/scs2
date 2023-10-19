package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartGroupModel;

public class YoCompositePattern
{
   private final String type;
   private final String[] componentIdentifiers;
   private final List<String[]> alternateComponentIdentifiers = new ArrayList<>();
   private final boolean crossRegistry;
   private final List<ChartGroupModel> preferredChartConfigurations;

   public static YoCompositePattern singleton(String type)
   {
      return new YoCompositePattern(type, false, null, new ArrayList<>());
   }

   public YoCompositePattern(String type, boolean crossRegistry, String[] componentIdentifiers, List<ChartGroupModel> preferredChartConfigurations)
   {
      this(type, crossRegistry, componentIdentifiers, Collections.emptyList(), preferredChartConfigurations);
   }
   public YoCompositePattern(String type, boolean crossRegistry, String[] componentIdentifiers, List<String> alternateComponentIdentifiers, List<ChartGroupModel> preferredChartConfigurations)
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
                                                  + "Problematic chart configuration: " + chartIdentifierList.getName() + " configuration size "
                                                  + chartIdentifierList.size() + " expected " + componentIdentifiers.length + ".");
            }
         }
      }

      this.componentIdentifiers = componentIdentifiers;
      this.preferredChartConfigurations = preferredChartConfigurations;
   }

   public void addAlternateComponentIdentifiers(String[] alternateComponentIdentifiers)
   {
      if (alternateComponentIdentifiers.length != componentIdentifiers.length)
         throw new IllegalArgumentException("Unexpected alternate component identifiers size for composite " + type + ".\n"
                                            + "The size of each alternate component identifiers should be equal to the number of components in the composite.\n"
                                            + "Problematic alternate component identifiers: " + Arrays.toString(alternateComponentIdentifiers) + " expected "
                                            + componentIdentifiers.length + ".");
      this.alternateComponentIdentifiers.add(alternateComponentIdentifiers);
   }

   public String getType()
   {
      return type;
   }

   public String[] getComponentIdentifiers()
   {
      return componentIdentifiers;
   }

   public List<String[]> getAlternateComponentIdentifiers()
   {
      return alternateComponentIdentifiers;
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