package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoDouble;

public class LinkedYoDouble extends LinkedYoVariable<YoDouble>
{
   LinkedYoDouble(YoDouble linkedVariable, YoDoubleBuffer buffer)
   {
      super(linkedVariable, buffer);
   }

   @Override
   DoublePullRequest toPullRequest()
   {
      return new DoublePullRequest(linkedYoVariable, buffer.getYoVariable().getValue());
   }

   @Override
   protected DoublePushRequest toPushRequest()
   {
      return new DoublePushRequest(linkedYoVariable.getValue(), buffer.getYoVariable());
   }

   @Override
   @SuppressWarnings("unchecked")
   public BufferSample<double[]> pollRequestedBufferSample()
   {
      return super.pollRequestedBufferSample();
   }

   @Override
   public String toString()
   {
      return String.format("%s: %s", linkedYoVariable.getName(), linkedYoVariable.getValue());
   }
}
