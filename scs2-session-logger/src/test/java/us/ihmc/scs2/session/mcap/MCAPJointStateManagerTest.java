package us.ihmc.scs2.session.mcap;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Covers the {@code sensor_msgs/msg/JointState} support added to {@link MCAPJointStateManager}: recovering the
 * {@code name[]} array's index ordering from a hand-encoded message, since that's the one piece the generic
 * {@link YoMCAPMessage} decoder can't recover on its own (string arrays aren't retained).
 */
public class MCAPJointStateManagerTest
{
   // Same schema McapLogConverter.JOINT_STATE_SCHEMA emits for the /joint_states channel.
   private static final String JOINT_STATE_SCHEMA =
         "std_msgs/Header header\n" +
         "string[] name\n" +
         "float64[] position\n" +
         "float64[] velocity\n" +
         "float64[] effort\n" +
         "\n================================================================================\n" +
         "MSG: std_msgs/Header\n" +
         "builtin_interfaces/Time stamp\n" +
         "string frame_id\n" +
         "\n================================================================================\n" +
         "MSG: builtin_interfaces/Time\n" +
         "int32 sec\n" +
         "uint32 nanosec\n";

   @Test
   public void testIsJointStateSchemaName()
   {
      assertTrue(MCAPJointStateManager.isJointStateSchemaName("sensor_msgs/msg/JointState"));
      assertTrue(MCAPJointStateManager.isJointStateSchemaName("sensor_msgs/JointState"));
      assertTrue(MCAPJointStateManager.isJointStateSchemaName("sensor_msgs::msg::dds_::JointState_"));
      assertFalse(MCAPJointStateManager.isJointStateSchemaName("tf2_msgs/msg/TFMessage"));
      assertFalse(MCAPJointStateManager.isJointStateSchemaName("foxglove::FrameTransform"));
   }

   @Test
   public void testReadNameArrayRecoversOrderPastHeader()
   {
      MCAPSchema loadedSchema = ROS2SchemaParser.loadSchema("sensor_msgs/msg/JointState", 1, JOINT_STATE_SCHEMA.getBytes(StandardCharsets.UTF_8));

      List<String> expectedNames = List.of("LEFT_HIP_X", "LEFT_HIP_Z", "RIGHT_HIP_X");
      Message message = encodeJointStateNamePrefix("odom", expectedNames);

      List<String> names = MCAPJointStateManager.readNameArray(loadedSchema, message);

      assertEquals(expectedNames, names);
   }

   /**
    * Encodes just enough of a JointState message (header + name[]) for {@link MCAPJointStateManager#readNameArray}
    * to recover the name ordering; position/velocity/effort aren't needed since that method stops right after
    * {@code name}.
    */
   private static Message encodeJointStateNamePrefix(String frameId, List<String> names)
   {
      ByteBuffer buf = ByteBuffer.allocate(1024).order(ByteOrder.LITTLE_ENDIAN);
      buf.put((byte) 0x00);
      buf.put((byte) 0x01);
      buf.put((byte) 0x00);
      buf.put((byte) 0x00);

      // header.stamp
      buf.putInt(0);
      buf.putInt(0);
      // header.frame_id
      writeString(buf, frameId);

      // name[]
      align(buf, 4);
      buf.putInt(names.size());
      for (String name : names)
         writeString(buf, name);

      byte[] bytes = new byte[buf.position()];
      buf.flip();
      buf.get(bytes);
      return new MutableMessage(0, bytes);
   }

   private static void writeString(ByteBuffer buf, String value)
   {
      align(buf, 4);
      byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
      buf.putInt(bytes.length + 1);
      buf.put(bytes);
      buf.put((byte) 0);
   }

   private static void align(ByteBuffer buf, int alignment)
   {
      int rem = (buf.position() - 4) % alignment; // CDR aligns from payload start, not buffer start.
      if (rem != 0)
      {
         int pad = alignment - rem;
         for (int i = 0; i < pad; i++)
            buf.put((byte) 0);
      }
   }
}
