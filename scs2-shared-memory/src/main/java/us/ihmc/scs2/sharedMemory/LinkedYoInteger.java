package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoInteger;

public class LinkedYoInteger extends LinkedYoVariable<YoInteger>
{
   LinkedYoInteger(YoInteger linkedVariable, YoIntegerBuffer buffer)
   {
      super(linkedVariable, buffer);
   }

   @Override
   IntegerPullRequest toPullRequest()
   {
      return new IntegerPullRequest(linkedYoVariable, buffer.getYoVariable().getValue());
   }

   @Override
   protected IntegerPushRequest toPushRequest()
   {
      return new IntegerPushRequest(linkedYoVariable.getValue(), buffer.getYoVariable());
   }

   @Override
   @SuppressWarnings("unchecked")
   public BufferSample<int[]> pollRequestedBufferSample()
   {
      return super.pollRequestedBufferSample();
   }

   @Override
   public String toString()
   {
      return String.format("%s: %s", linkedYoVariable.getName(), linkedYoVariable.getValue());
   }
}
