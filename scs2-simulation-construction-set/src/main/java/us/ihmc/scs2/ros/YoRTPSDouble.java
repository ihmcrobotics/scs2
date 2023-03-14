package us.ihmc.scs2.ros;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

import std_msgs.msg.dds.Float64;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoRTPSDouble
{
   public enum RTPS_MODE
   {
      PUBLISH_ONLY, SUBSCRIBE_ONLY, BOTH
   };

   private final SimulationConstructionSet2 scs;

   public YoRTPSDouble(ROS2Node node, SimulationConstructionSet2 scs, YoRegistry parentRegistry, String topic, RTPS_MODE mode)
   {
      this.scs = scs;
      String[] topicSplit = topic.split("/");
      String[] namespaces = Arrays.copyOfRange(topicSplit, 0, topicSplit.length - 1);

      String topicName = topicSplit[topicSplit.length - 1];
      YoRegistry registry = YoVariableTools.getOrCreateRegistry(parentRegistry, namespaces);

      final YoDouble variable = YoVariableTools.getOrCreateYoDouble(registry, topicName);

      if (mode == RTPS_MODE.SUBSCRIBE_ONLY || mode == RTPS_MODE.BOTH)
      {
         attachYoSubscriber(node, topic, topicName, variable);
      }

      if (mode == RTPS_MODE.PUBLISH_ONLY || mode == RTPS_MODE.BOTH)
      {
         attachYoPublisher(node, topic, topicName, variable);
      }
   }

   private void attachYoSubscriber(ROS2Node node, String topic, String topicName, final YoDouble variable)
   {
      try
      {
         System.out.println("Binding Subscriber:  " + topicName + " to " + topic);
         node.createSubscription(new std_msgs.msg.dds.Float64PubSubType(), sub ->
         {
            if (scs.isSimulating())
            {
               Float64 rosData = sub.takeNextData();
               variable.set(rosData.getData());
               //               System.out.println(topicName + " : " + rosData.getData());
            }

         }, topic);
      }
      catch (IOException e)
      {
         String error = topic + " " + topicName + " error";
         System.out.println(error);
         e.printStackTrace();
      }
   }

   private void attachYoPublisher(ROS2Node node, String topic, String topicName, final YoDouble variable)
   {
      System.out.println("Binding Publisher:  " + topic + " to " + topicName);
      ROS2PublisherBasics<Float64> publisher;
      try
      {
         publisher = node.createPublisher(new std_msgs.msg.dds.Float64PubSubType(), topic);
         variable.addListener(new YoVariableChangedListener()
         {
            @Override
            public void changed(YoVariable source)
            {
               if (scs.isSimulating())
               {
                  //               System.out.println(topicName + " : " + source.getValueAsDouble());
                  Float64 data = new Float64();
                  data.setData(source.getValueAsDouble());
                  publisher.publish(data);
               }
            }
         });
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
