package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoDouble;

public class DoublePushRequest implements PushRequest<YoDouble>
{
   private final double valueToPush;
   private final YoDouble variableToUpdate;

   public DoublePushRequest(double valueToPush, YoDouble variableToUpdate)
   {
      this.valueToPush = valueToPush;
      this.variableToUpdate = variableToUpdate;
   }

   @Override
   public boolean push()
   {
      if (valueToPush == variableToUpdate.getValue())
         return false;

      variableToUpdate.set(valueToPush);

      return true;
   }

   YoDouble getVariableToUpdate()
   {
      return variableToUpdate;
   }

   double getValueToPush()
   {
      return valueToPush;
   }
}
