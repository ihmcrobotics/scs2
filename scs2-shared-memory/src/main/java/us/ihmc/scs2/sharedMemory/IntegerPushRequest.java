package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoInteger;

public class IntegerPushRequest implements PushRequest<YoInteger>
{
   private final int valueToPush;
   private final YoInteger variableToUpdate;

   public IntegerPushRequest(int valueToPush, YoInteger variableToUpdate)
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

   YoInteger getVariableToUpdate()
   {
      return variableToUpdate;
   }

   int getValueToPush()
   {
      return valueToPush;
   }
}
