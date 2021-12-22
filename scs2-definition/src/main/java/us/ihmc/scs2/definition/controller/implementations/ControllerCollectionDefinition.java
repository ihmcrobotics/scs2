package us.ihmc.scs2.definition.controller.implementations;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;

public class ControllerCollectionDefinition implements ControllerDefinition
{
   private String controllerName;
   private List<ControllerDefinition> controllerDefinitions;

   public ControllerCollectionDefinition setControllerName(String controllerName)
   {
      this.controllerName = controllerName;
      return this;
   }

   public ControllerCollectionDefinition addControllerOutputReset()
   {
      if (controllerDefinitions == null)
         controllerDefinitions = new ArrayList<>();
      controllerDefinitions.add(ResetControllerOutputController.newControllerDefinition());
      return this;
   }

   public ControllerCollectionDefinition addControllerDefinition(ControllerDefinition controllerDefinition)
   {
      if (controllerDefinitions == null)
         controllerDefinitions = new ArrayList<>();
      controllerDefinitions.add(controllerDefinition);
      return this;
   }

   public ControllerCollectionDefinition addControllerDefinitions(ControllerDefinition... controllerDefinitions)
   {
      for (ControllerDefinition controllerDefinition : controllerDefinitions)
         addControllerDefinition(controllerDefinition);
      return this;
   }

   public ControllerCollectionDefinition addControllerDefinitions(Iterable<? extends ControllerDefinition> controllerDefinitions)
   {
      for (ControllerDefinition controllerDefinition : controllerDefinitions)
         addControllerDefinition(controllerDefinition);
      return this;
   }

   @Override
   public Controller newController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      Objects.requireNonNull(controllerName);
      Objects.requireNonNull(controllerDefinitions);

      Controller[] controllers = controllerDefinitions.stream().map(def -> def.newController(controllerInput, controllerOutput)).toArray(Controller[]::new);
      return new ControllerCollection(controllerName, controllers);
   }
}
