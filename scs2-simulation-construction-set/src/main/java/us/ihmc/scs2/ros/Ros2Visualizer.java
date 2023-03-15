package us.ihmc.scs2.ros;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Parameter;
import com.martiansoftware.jsap.SimpleJSAP;

import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.ros.YoRTPSDouble.RTPS_MODE;
import us.ihmc.yoVariables.registry.YoRegistry;

public class Ros2Visualizer
{
   private SimulationConstructionSet2 scs;

   public Ros2Visualizer(int domainID, List<String> subTopics, List<String> pubTopics) throws IOException, InterruptedException
   {
      //try and give resource loading a chance to catch up
      Thread.sleep(1500);
      scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory());
      
      //try and give resource loading a chance to catch up
      Thread.sleep(200);

      //      * Create a new ROS2 node.
      //      *
      //      * @param Domain             DDS domain to use. Use DomainFactory.getDomain(implementation)
      //      * @param name               Name for the node
      //      * @param namespace          namespace for the ros node i.e. DDS partition
      //      * @param domainId           Domain ID for the ros node
      //      * @param addressRestriction Restrict network traffic to the given addresses. When provided, it
      //      *                           should describe one of the addresses of the computer hosting this node.
      //      *                           Optional.
      ROS2Node node = new ROS2Node(DomainFactory.getDomain(PubSubImplementation.FAST_RTPS), "SCSVisualizer", "/bw", domainID);

      YoRegistry rootRegistry = scs.getRootRegistry();

      for (String topic : subTopics)
      {
         new YoRTPSDouble(node, scs,  rootRegistry, topic, RTPS_MODE.SUBSCRIBE_ONLY);
      }

      for (String topic : pubTopics)
      {
         new YoRTPSDouble(node, scs,  rootRegistry, topic, RTPS_MODE.PUBLISH_ONLY);
      }

      //      new DoubleRTPSPubSubManager2(scs, node, subTopics, pubTopics,  rootRegistry);

      
      scs.addVisualizerShutdownListener(new Runnable()
      {
         @Override
         public void run()
         {
            node.destroy();
         }
      });
      
      scs.setRealTimeRateSimulation(true);
      scs.start(true, false, false);
   }

   public static void main(String[] args) throws IOException, JSAPException, InterruptedException
   {
      FlaggedOption domainIdFlag = new FlaggedOption("domain");
      domainIdFlag.setRequired(true);
      domainIdFlag.setShortFlag('d');
      domainIdFlag.setLongFlag("domain");
      domainIdFlag.setHelp("Ros Domain ID, Should match the domain ID set on the ROS2 Master. Example: -d 93 ");

      FlaggedOption publishTopicsFlag = new FlaggedOption("publish");
      publishTopicsFlag.setRequired(true);
      publishTopicsFlag.setShortFlag('p');
      publishTopicsFlag.setLongFlag("publish");
      publishTopicsFlag.setHelp("Please specify the file that contains the topics you would like to publish data to. Example: -p publishTopics.txt ");

      FlaggedOption subscribeTopicsFlag = new FlaggedOption("subscribe");
      subscribeTopicsFlag.setRequired(true);
      subscribeTopicsFlag.setShortFlag('s');
      subscribeTopicsFlag.setLongFlag("subscribe");
      subscribeTopicsFlag.setHelp("Please specify the file that contains the topics you would like to subscribe data to. Example: -s subscribeTopics.txt ");

      SimpleJSAP jsap = new SimpleJSAP("Ros2Visualizer", "Help", new Parameter[] {domainIdFlag, publishTopicsFlag, subscribeTopicsFlag});
      JSAPResult config = jsap.parse(args);

      if (jsap.messagePrinted())
      {
         System.out.println(jsap.getHelp());
         System.exit(-1);
      }

      int domainID = Integer.parseInt(config.getString("domain"));
      String subFileName = config.getString("subscribe");
      String pubFileName = config.getString("publish");

      Path subFilePath = new File("./" + subFileName).toPath();

      List<String> subTopics = Files.readAllLines(subFilePath);

      Path pubFilePath = new File("./" + pubFileName).toPath();
      List<String> pubTopics = Files.readAllLines(pubFilePath);

      LogTools.info("Staring the Ros 2 Visualizer using Ros Domain ID: " + domainID);

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
      
      new Ros2Visualizer(domainID, subTopics, pubTopics);
   }

}
