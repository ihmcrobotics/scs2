package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoInteger;

public class IntegerPullRequest implements PullRequest<YoInteger>
{
   private final YoInteger variableToUpdate;
   private final int valueToPull;

   public IntegerPullRequest(YoInteger variableToUpdate, int valueToPull)
   {
      this.variableToUpdate = variableToUpdate;
      this.valueToPull = valueToPull;
   }

   @Override
   public void pull()
   {
      variableToUpdate.set(valueToPull);
   }

   YoInteger getVariableToUpdate()
   {
      return variableToUpdate;
   }

   public int getValueToPull()
   {
      return valueToPull;
   }
}
