package us.ihmc.scs2.definition.controller.interfaces;

import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;

public interface ControllerDefinition
{
   Controller newController(ControllerInput controllerInput, ControllerOutput controllerOutput);

   public static ControllerDefinition emptyControllerDefinition()
   {
      return (input, output) -> Controller.emptyController();
   }
}
