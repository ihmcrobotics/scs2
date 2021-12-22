package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.pattern;

import java.util.HashSet;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.YoChartGroupModelEditorController;

public class YoCompositePatternControllerTools
{

   public static final int YO_COMPOSITE_CUSTOM_PATTERN_MIN_NUMBER_OF_COMPONENTS = 2;

   public static boolean areComponentIdentifierNamesValid(List<? extends String> componentIdentifierNames)
   {
      if (componentIdentifierNames == null || componentIdentifierNames.size() < YO_COMPOSITE_CUSTOM_PATTERN_MIN_NUMBER_OF_COMPONENTS)
         return false;
      return !YoCompositePatternControllerTools.isAnyEmpty(componentIdentifierNames) && YoCompositePatternControllerTools.areAllUnique(componentIdentifierNames);
   }

   public static boolean areChartGroupModelNamesValid(List<? extends YoChartGroupModelEditorController> controllerList)
   {
      if (controllerList.isEmpty())
         return true;
   
      List<String> names = controllerList.stream().map(controller -> controller.configurationNameProperty().get()).collect(Collectors.toList());
   
      return !YoCompositePatternControllerTools.isAnyEmpty(names) && YoCompositePatternControllerTools.areAllUnique(names);
   }

   public static boolean isAnyEmpty(List<? extends String> stringList)
   {
      return stringList.stream().anyMatch(String::isEmpty);
   }

   public static boolean areAllUnique(List<? extends String> stringList)
   {
      return new HashSet<>(stringList).size() == stringList.size();
   }

   public static String getValidityStyleBorder(boolean valid)
   {
      if (valid)
         return "-fx-border-width:3;-fx-border-color:#89e0c0";
      else
         return "-fx-border-width:3;-fx-border-color:#edafb7";
   }

}
