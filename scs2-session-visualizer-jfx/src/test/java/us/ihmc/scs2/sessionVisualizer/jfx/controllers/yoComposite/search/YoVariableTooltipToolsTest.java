package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoVariableTooltipToolsTest
{
   @Test
   public void testCreateYoVariableTooltipTextIncludesDescription()
   {
      YoRegistry registry = new YoRegistry("root");
      YoDouble yoVariable = new YoDouble("testVariable", "Useful description for hover text.", registry);

      assertEquals("testVariable\nroot\n\nUseful description for hover text.", YoVariableTooltipTools.createYoVariableTooltipText(yoVariable));
   }

   @Test
   public void testCreateYoVariableTooltipTextHandlesBlankDescription()
   {
      YoRegistry registry = new YoRegistry("root");
      YoDouble yoVariable = new YoDouble("testVariable", "   ", registry);

      assertEquals("testVariable\nroot", YoVariableTooltipTools.createYoVariableTooltipText(yoVariable));
   }

   @Test
   public void testCreateYoVariableTooltipTextHandlesNullDescription()
   {
      YoRegistry registry = new YoRegistry("root");
      YoDouble yoVariable = new YoDouble("testVariable", null, registry);

      assertEquals("testVariable\nroot", YoVariableTooltipTools.createYoVariableTooltipText(yoVariable));
   }
}
