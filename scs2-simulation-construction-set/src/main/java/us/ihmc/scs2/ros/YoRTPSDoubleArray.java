package us.ihmc.scs2.ros;

import us.ihmc.ros2.ROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.ros.YoRTPSDouble.RTPS_MODE;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoRTPSDoubleArray
{
   public YoRTPSDoubleArray(ROS2Node node, SimulationConstructionSet2 scs,  String topic, int numberOfElements, YoRegistry parentRegistry, RTPS_MODE mode)
   {
      for (int i = 0; i < numberOfElements; i++)
      {
         new YoRTPSDouble(node, scs, parentRegistry, topic, mode);
      }
   }
}
