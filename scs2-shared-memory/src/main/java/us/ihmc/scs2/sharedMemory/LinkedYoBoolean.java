package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoBoolean;

public class LinkedYoBoolean extends LinkedYoVariable<YoBoolean>
{
   LinkedYoBoolean(YoBoolean linkedVariable, YoBooleanBuffer buffer, Object initialUser)
   {
      super(linkedVariable, buffer, initialUser);
   }

   @Override
   BooleanPullRequest toPullRequest()
   {
      return new BooleanPullRequest(linkedYoVariable, buffer.getYoVariable().getValue());
   }

   @Override
   protected BooleanPushRequest toPushRequest()
   {
      return new BooleanPushRequest(linkedYoVariable.getValue(), buffer.getYoVariable());
   }

   @Override
   @SuppressWarnings("unchecked")
   public BufferSample<boolean[]> pollRequestedBufferSample()
   {
      return super.pollRequestedBufferSample();
   }

   @Override
   public String toString()
   {
      return String.format("%s: %s", linkedYoVariable.getName(), linkedYoVariable.getValue());
   }
}