package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoLong;

public class LongPushRequest implements PushRequest<YoLong>
{
   private final long valueToPush;
   private final YoLong variableToUpdate;

   public LongPushRequest(long valueToPush, YoLong variableToUpdate)
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

   YoLong getVariableToUpdate()
   {
      return variableToUpdate;
   }

   long getValueToPush()
   {
      return valueToPush;
   }
}
