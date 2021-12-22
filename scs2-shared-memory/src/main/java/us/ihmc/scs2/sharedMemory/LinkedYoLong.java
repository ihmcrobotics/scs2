package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoLong;

public class LinkedYoLong extends LinkedYoVariable<YoLong>
{
   LinkedYoLong(YoLong linkedVariable, YoLongBuffer buffer)
   {
      super(linkedVariable, buffer);
   }

   @Override
   LongPullRequest toPullRequest()
   {
      return new LongPullRequest(linkedYoVariable, buffer.getYoVariable().getValue());
   }

   @Override
   protected LongPushRequest toPushRequest()
   {
      return new LongPushRequest(linkedYoVariable.getValue(), buffer.getYoVariable());
   }

   @Override
   @SuppressWarnings("unchecked")
   public BufferSample<long[]> pollRequestedBufferSample()
   {
      return super.pollRequestedBufferSample();
   }

   @Override
   public String toString()
   {
      return String.format("%s: %s", linkedYoVariable.getName(), linkedYoVariable.getValue());
   }
}
