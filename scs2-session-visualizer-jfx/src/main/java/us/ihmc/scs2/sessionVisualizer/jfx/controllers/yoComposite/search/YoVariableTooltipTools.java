package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import us.ihmc.yoVariables.variable.YoVariable;

final class YoVariableTooltipTools
{
   private YoVariableTooltipTools()
   {
   }

   static String createYoVariableTooltipText(YoVariable yoVariable)
   {
      String tooltipText = yoVariable.getName() + "\n" + yoVariable.getNamespace();
      String description = yoVariable.getDescription();

      if (description != null && !description.isBlank())
         tooltipText += "\n\n" + description;

      return tooltipText;
   }
}
