package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoBoolean;

public class BooleanPushRequest implements PushRequest<YoBoolean>
{
   private final boolean valueToPush;
   private final YoBoolean variableToUpdate;

   public BooleanPushRequest(boolean valueToPush, YoBoolean variableToUpdate)
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

   YoBoolean getVariableToUpdate()
   {
      return variableToUpdate;
   }

   boolean getValueToPush()
   {
      return valueToPush;
   }
}
