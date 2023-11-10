package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class YoEquationManager
{
   private final YoRegistry rootRegistry;
   private final YoRegistry equationRegistry = new YoRegistry("equations");

   private final List<Equation> equations = new ArrayList<>();

   public YoEquationManager(YoRegistry rootRegistry)
   {
      this.rootRegistry = rootRegistry;
      rootRegistry.addChild(equationRegistry);
   }

   public void addEquation(YoEquationDefinition equationDefinition)
   {
   }

   public void update()
   {
   }

   public List<YoEquationDefinition> getEquationDefinitions()
   {
      return equations.stream().map(Equation::toYoEquationDefinition).toList();
   }
}
