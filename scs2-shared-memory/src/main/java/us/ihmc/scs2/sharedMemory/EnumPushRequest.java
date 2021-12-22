package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoEnum;

public class EnumPushRequest<E extends Enum<E>> implements PushRequest<YoEnum<E>>
{
   private final int valueToPush;
   private final YoEnum<E> variableToUpdate;

   public EnumPushRequest(int valueToPush, YoEnum<E> variableToUpdate)
   {
      this.valueToPush = valueToPush;
      this.variableToUpdate = variableToUpdate;
   }

   @Override
   public boolean push()
   {
      if (valueToPush == variableToUpdate.getOrdinal())
         return false;

      variableToUpdate.set(valueToPush);

      return true;
   }

   YoEnum<E> getVariableToUpdate()
   {
      return variableToUpdate;
   }

   int getValueToPush()
   {
      return valueToPush;
   }
}
