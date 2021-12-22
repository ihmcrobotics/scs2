package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoDouble;

public class DoublePullRequest implements PullRequest<YoDouble>
{
   private final YoDouble variableToUpdate;
   private final double valueToPull;

   public DoublePullRequest(YoDouble variableToUpdate, double valueToPull)
   {
      this.variableToUpdate = variableToUpdate;
      this.valueToPull = valueToPull;
   }

   @Override
   public void pull()
   {
      variableToUpdate.set(valueToPull);
   }

   YoDouble getVariableToUpdate()
   {
      return variableToUpdate;
   }

   public double getValueToPull()
   {
      return valueToPull;
   }
}
