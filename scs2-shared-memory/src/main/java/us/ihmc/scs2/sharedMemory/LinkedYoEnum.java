package us.ihmc.scs2.sharedMemory;

import us.ihmc.yoVariables.variable.YoEnum;

public class LinkedYoEnum<E extends Enum<E>> extends LinkedYoVariable<YoEnum<E>>
{
   LinkedYoEnum(YoEnum<E> linkedVariable, YoEnumBuffer<E> buffer)
   {
      super(linkedVariable, buffer);
   }

   @Override
   EnumPullRequest<E> toPullRequest()
   {
      return new EnumPullRequest<>(linkedYoVariable, buffer.getYoVariable().getOrdinal());
   }

   @Override
   protected EnumPushRequest<E> toPushRequest()
   {
      return new EnumPushRequest<>(linkedYoVariable.getOrdinal(), buffer.getYoVariable());
   }

   @Override
   @SuppressWarnings("unchecked")
   public BufferSample<byte[]> pollRequestedBufferSample()
   {
      return super.pollRequestedBufferSample();
   }

   @Override
   public String toString()
   {
      return String.format("%s: %s", linkedYoVariable.getName(), linkedYoVariable.getStringValue());
   }
}
