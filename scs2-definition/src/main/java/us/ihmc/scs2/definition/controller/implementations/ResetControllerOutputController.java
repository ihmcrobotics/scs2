package us.ihmc.scs2.definition.controller.implementations;

import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;

public class ResetControllerOutputController implements Controller
{
   private final ControllerOutput controllerOutput;

   public ResetControllerOutputController(ControllerOutput controllerOutput)
   {
      this.controllerOutput = controllerOutput;
   }

   @Override
   public void doControl()
   {
      controllerOutput.clear();
   }

   public static ControllerDefinition newControllerDefinition()
   {
      return (input, output) -> new ResetControllerOutputController(output);
   }
}
