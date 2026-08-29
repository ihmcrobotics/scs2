package us.ihmc.scs2.session;

public class SensorMessage<T>
{
   private final String robotName;
   private final String sensorName;
   private final T messageContent;

   public SensorMessage(String robotName, String sensorName, T messageContent)
   {
      this.robotName = robotName;
      this.sensorName = sensorName;
      this.messageContent = messageContent;
   }

   public String getRobotName()
   {
      return robotName;
   }

   public String getSensorName()
   {
      return sensorName;
   }

   public T getMessageContent()
   {
      return messageContent;
   }

   @Override
   public String toString()
   {
      return "[robotName=" + robotName + ", sensorName=" + sensorName + ", messageContent=" + messageContent + "]";
   }
}
