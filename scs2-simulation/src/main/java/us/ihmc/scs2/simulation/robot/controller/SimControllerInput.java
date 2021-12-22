package us.ihmc.scs2.simulation.robot.controller;

import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimMultiBodySystemReadOnly;

public class SimControllerInput extends ControllerInput
{

   public SimControllerInput(SimMultiBodySystemReadOnly input)
   {
      super(input);
   }

   @Override
   public SimMultiBodySystemReadOnly getInput()
   {
      return (SimMultiBodySystemReadOnly) super.getInput();
   }
}
