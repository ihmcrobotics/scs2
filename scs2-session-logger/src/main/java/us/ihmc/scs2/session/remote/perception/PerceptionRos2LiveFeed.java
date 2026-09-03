package us.ihmc.scs2.session.remote.perception;

import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2SubscriptionCallbackSampler;
import us.ihmc.jros2.ROS2Topic;

import java.io.Closeable;

/**
 * Owns a single {@link ROS2Node} for the live viewer, shared by any number of perception sources (a height map
 * today, potentially a voxel map or multiple height maps later) each subscribing via {@link #subscribe}. Mirrors
 * {@code PerceptionMcapLogger}'s multi-channel design (one node, N independent subscriptions) on the write side -
 * a new live source is one more {@link #subscribe} call here, not a new node.
 * <p>
 * Constructed once a live {@code RemoteSession} is fully connected, closed when that session ends.
 */
public class PerceptionRos2LiveFeed implements Closeable
{
   private final ROS2Node ros2Node;

   public PerceptionRos2LiveFeed()
   {
      ros2Node = new ROS2Node("scs2_perception_live_feed_node");
   }

   public <T extends ROS2Message<T>> void subscribe(ROS2Topic<T> topic, ROS2SubscriptionCallbackSampler<T> callback)
   {
      ros2Node.createSubscriptionSampler(topic, callback);
   }

   @Override
   public void close()
   {
      ros2Node.close();
   }
}
