package us.ihmc.scs2.session;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.symbolic.YoEquationManager.YoEquationListChange;

import java.awt.image.BufferedImage;

public class SessionMessagerAPI
{
   private SessionMessagerAPI()
   {
   }

   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final Category root = apiFactory.createRootCategory("SessionAPI");

   private static final CategoryTheme Session = apiFactory.createCategoryTheme("Session");
   private static final CategoryTheme Record = apiFactory.createCategoryTheme("Record");
   private static final CategoryTheme Export = apiFactory.createCategoryTheme("Export");
   private static final CategoryTheme Initialize = apiFactory.createCategoryTheme("Initialize");

   private static final CategoryTheme Run = apiFactory.createCategoryTheme("Run");
   private static final CategoryTheme Playback = apiFactory.createCategoryTheme("Playback");
   private static final CategoryTheme Change = apiFactory.createCategoryTheme("Change");
   private static final CategoryTheme RobotDefinition = apiFactory.createCategoryTheme("RobotDefinition");
   private static final CategoryTheme Equation = apiFactory.createCategoryTheme("Equation");

   private static final TopicTheme RealTimeRate = apiFactory.createTopicTheme("RealTimeRate");
   private static final TopicTheme MaxDuration = apiFactory.createTopicTheme("MaxDuration");
   private static final TopicTheme Period = apiFactory.createTypedTopicTheme("Period");
   private static final TopicTheme State = apiFactory.createTypedTopicTheme("State");
   private static final TopicTheme Mode = apiFactory.createTypedTopicTheme("Mode");
   private static final TopicTheme TickPeriod = apiFactory.createTypedTopicTheme("TickPeriod");
   private static final TopicTheme Data = apiFactory.createTypedTopicTheme("Data");
   private static final TopicTheme Request = apiFactory.createTypedTopicTheme("Request");

   public static final Topic<Long> SessionDTNanoseconds = root.child(Session).child(Run).topic(Period);
   public static final Topic<SessionState> SessionCurrentState = root.child(Session).topic(State);
   public static final Topic<SessionMode> SessionCurrentMode = root.child(Session).topic(Mode);
   public static final Topic<Integer> BufferRecordTickPeriod = root.child(Session).child(Record).topic(TickPeriod);
   public static final Topic<Integer> InitializeBufferRecordTickPeriod = root.child(Session).child(Initialize).child(Record).topic(TickPeriod);

   public static final Topic<Boolean> RunAtRealTimeRate = root.child(Session).child(Run).topic(RealTimeRate);
   public static final Topic<Double> PlaybackRealTimeRate = root.child(Session).child(Playback).topic(RealTimeRate);

   public static final Topic<Long> RunMaxDuration = root.child(Session).child(Run).topic(MaxDuration);

   public static final Topic<SessionDataExportRequest> SessionDataExportRequest = root.child(Session).child(Export).topic(Data);
   public static final Topic<SessionRobotDefinitionListChange> SessionRobotDefinitionListChangeRequest = root.child(Session)
                                                                                                             .child(Change)
                                                                                                             .child(RobotDefinition)
                                                                                                             .topic(Request);
   public static final Topic<SessionRobotDefinitionListChange> SessionRobotDefinitionListChangeState = root.child(Session)
                                                                                                           .child(Change)
                                                                                                           .child(RobotDefinition)
                                                                                                           .topic(State);
   public static final Topic<YoEquationListChange> SessionYoEquationListChangeRequest = root.child(Session).child(Change).child(Equation).topic(Request);
   public static final Topic<YoEquationListChange> SessionYoEquationListChangeState = root.child(Session).child(Change).child(Equation).topic(State);

   static
   { // Ensure that the Sensors is loaded before closing the API.
      new Sensors();
   }

   public static class Sensors
   {
      private static final CategoryTheme Sensor = apiFactory.createCategoryTheme("Sensor");
      private static final CategoryTheme Camera = apiFactory.createCategoryTheme("Camera");

      private static final TopicTheme Definition = apiFactory.createTypedTopicTheme("Definition");
      private static final TopicTheme Frame = apiFactory.createTypedTopicTheme("Frame");

      public static final Topic<SensorMessage<CameraSensorDefinition>> CameraSensorDefinitionData = root.child(Sensor).child(Camera).topic(Definition);
      public static final Topic<SensorMessage<BufferedImage>> CameraSensorFrame = root.child(Sensor).child(Camera).topic(Frame);

      public static class SensorMessage<T>
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
   }

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
