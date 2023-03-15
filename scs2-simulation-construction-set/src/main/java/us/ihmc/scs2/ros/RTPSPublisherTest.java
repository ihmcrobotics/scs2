package us.ihmc.scs2.ros;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Random;
import java.util.concurrent.TimeUnit;

import com.martiansoftware.jsap.JSAPException;

import std_msgs.msg.dds.Float64;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.util.PeriodicGCFreeNonRealtimeThreadSchedulerFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.util.PeriodicThreadSchedulerFactory;

public class RTPSPublisherTest
{
   private final long startTime = System.currentTimeMillis();
   public RTPSPublisherTest(int domainID, List<String> subTopics, List<String> pubTopics)
   {
      try
      {
         ROS2Node node = new ROS2Node(DomainFactory.getDomain(PubSubImplementation.FAST_RTPS), "RTPSPublisherTest", "/bw", domainID);

         for (String topic : pubTopics)
         {
            createPublisher(node, topic);
         }

         for (String topic : subTopics)
         {
            createSubscriber(node, topic);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void createSubscriber(ROS2Node node, String topic)
   {
      System.out.println("Binding Subscriber:  " + topic);
      try
      {
         node.createSubscription(new std_msgs.msg.dds.Float64PubSubType(), sub ->
         {
            {
               Float64 rosData = sub.takeNextData();
               System.out.println(topic + " : " + rosData.getData());
            }

         }, topic);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void createPublisher(ROS2Node node, String topic)
   {
      System.out.println("Binding Publisher:  " + topic + " to " + topic);
      ROS2PublisherBasics<Float64> publisher;
      try
      {
         publisher = node.createPublisher(new std_msgs.msg.dds.Float64PubSubType(), topic);
         PeriodicThreadSchedulerFactory factory = new PeriodicGCFreeNonRealtimeThreadSchedulerFactory();
         PeriodicThreadScheduler threadScheduler = factory.createPeriodicThreadScheduler("");
         
         Random rand = new Random();
         int seed = rand.nextInt(50);
         Runnable r = new Runnable()
         {
            @Override
            public void run()
            {
               long elapsed = System.currentTimeMillis() - startTime; 
               double elapsedSeconds = elapsed / 1000.0;
               
               Float64 data = new Float64();
               data.setData(seed + elapsedSeconds);
               publisher.publish(data);
            }
         };
         
         threadScheduler.schedule(r, 100, TimeUnit.MILLISECONDS);
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public static void main(String[] args) throws IOException, JSAPException
   {

      System.out.println("Starting");
      
      int domainID = 5;
      String subFileName = "./publishTopics.txt";
      String pubFileName = "./subscribeTopics.txt";

      Path subFilePath = new File(subFileName).toPath();
      List<String> subTopics = Files.readAllLines(subFilePath);

      Path pubFilePath = new File(pubFileName).toPath();
      List<String> pubTopics = Files.readAllLines(pubFilePath);

      LogTools.info("Staring the Ros 2 Publisher using Ros Domain ID: " + domainID);

      LogTools.info("Creating Publish YoVariables for the following Topics:");
      for (String topic : pubTopics)
      {
         System.out.println(topic);
      }

      LogTools.info("Creating Subscribe YoVariables for the following Topics:");
      for (String topic : subTopics)
      {
         System.out.println(topic);
      }

      new RTPSPublisherTest(domainID, subTopics, pubTopics);
   }
}
