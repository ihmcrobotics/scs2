package us.ihmc.scs2.session.mcap;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.MCAPOdometryManager.OdometryTwist;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Covers the {@code nav_msgs/msg/Odometry} support in {@link MCAPOdometryManager}: schema-name matching, and
 * decoding the exact twist from a hand-encoded message - including correctly skipping past the {@code pose} field,
 * whose nested {@code PoseWithCovariance.pose} (a {@code Pose}) has the same name as its container, which is
 * exactly why this channel can't go through {@link YoMCAPMessage}'s generic decoder (see class javadoc on
 * {@link MCAPOdometryManager}).
 */
public class MCAPOdometryManagerTest
{
   // Same schema McapLogConverter.ODOMETRY_SCHEMA emits for the /odom channel.
   private static final String ODOMETRY_SCHEMA =
         "std_msgs/Header header\n" +
         "string child_frame_id\n" +
         "geometry_msgs/PoseWithCovariance pose\n" +
         "geometry_msgs/TwistWithCovariance twist\n" +
         "\n================================================================================\n" +
         "MSG: std_msgs/Header\n" +
         "builtin_interfaces/Time stamp\n" +
         "string frame_id\n" +
         "\n================================================================================\n" +
         "MSG: builtin_interfaces/Time\n" +
         "int32 sec\n" +
         "uint32 nanosec\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/PoseWithCovariance\n" +
         "geometry_msgs/Pose pose\n" +
         "float64[36] covariance\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Pose\n" +
         "geometry_msgs/Point position\n" +
         "geometry_msgs/Quaternion orientation\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Point\n" +
         "float64 x\n" +
         "float64 y\n" +
         "float64 z\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Quaternion\n" +
         "float64 x\n" +
         "float64 y\n" +
         "float64 z\n" +
         "float64 w\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/TwistWithCovariance\n" +
         "geometry_msgs/Twist twist\n" +
         "float64[36] covariance\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Twist\n" +
         "geometry_msgs/Vector3 linear\n" +
         "geometry_msgs/Vector3 angular\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Vector3\n" +
         "float64 x\n" +
         "float64 y\n" +
         "float64 z\n";

   @Test
   public void testIsOdometrySchemaName()
   {
      assertTrue(MCAPOdometryManager.isOdometrySchemaName("nav_msgs/msg/Odometry"));
      assertTrue(MCAPOdometryManager.isOdometrySchemaName("nav_msgs/Odometry"));
      assertTrue(MCAPOdometryManager.isOdometrySchemaName("nav_msgs::msg::dds_::Odometry_"));
      assertFalse(MCAPOdometryManager.isOdometrySchemaName("tf2_msgs/msg/TFMessage"));
      assertFalse(MCAPOdometryManager.isOdometrySchemaName("sensor_msgs/msg/JointState"));
   }

   @Test
   public void testReadTwistSkipsPoseAndRecoversExactTwist()
   {
      MCAPSchema loadedSchema = ROS2SchemaParser.loadSchema("nav_msgs/msg/Odometry", 1, ODOMETRY_SCHEMA.getBytes(StandardCharsets.UTF_8));

      double expectedAngularX = 0.1, expectedAngularY = -0.2, expectedAngularZ = 0.3;
      double expectedLinearX = 1.5, expectedLinearY = -2.5, expectedLinearZ = 0.05;
      byte[] messageBytes = encodeOdometryMessage(expectedAngularX, expectedAngularY, expectedAngularZ, expectedLinearX, expectedLinearY, expectedLinearZ);

      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(ByteBuffer.wrap(messageBytes).order(ByteOrder.LITTLE_ENDIAN), 0, messageBytes.length);
      OdometryTwist twist;
      try
      {
         twist = MCAPOdometryManager.readTwist(cdr, loadedSchema, loadedSchema.getSubSchemaMap());
      }
      finally
      {
         cdr.finalize(true);
      }

      assertEquals(expectedAngularX, twist.angularX(), 1e-12);
      assertEquals(expectedAngularY, twist.angularY(), 1e-12);
      assertEquals(expectedAngularZ, twist.angularZ(), 1e-12);
      assertEquals(expectedLinearX, twist.linearX(), 1e-12);
      assertEquals(expectedLinearY, twist.linearY(), 1e-12);
      assertEquals(expectedLinearZ, twist.linearZ(), 1e-12);
   }

   /**
    * Encodes a full {@code nav_msgs/Odometry} message (header, child_frame_id, pose+covariance, twist+covariance) -
    * the pose fields are non-zero too, to make sure a mistaken skip-offset would actually be caught by the assertions.
    */
   private static byte[] encodeOdometryMessage(double angularX, double angularY, double angularZ, double linearX, double linearY, double linearZ)
   {
      ByteBuffer buf = ByteBuffer.allocate(2048).order(ByteOrder.LITTLE_ENDIAN);
      buf.put((byte) 0x00);
      buf.put((byte) 0x01);
      buf.put((byte) 0x00);
      buf.put((byte) 0x00);

      // header.stamp
      buf.putInt(12);
      buf.putInt(34);
      // header.frame_id
      writeString(buf, "map");
      // child_frame_id
      writeString(buf, "PELVIS_LINK");

      // pose.pose.position
      align(buf, 8);
      buf.putDouble(9.1);
      buf.putDouble(9.2);
      buf.putDouble(9.3);
      // pose.pose.orientation
      buf.putDouble(0.0);
      buf.putDouble(0.0);
      buf.putDouble(0.0);
      buf.putDouble(1.0);
      // pose.covariance[36]
      for (int i = 0; i < 36; i++)
         buf.putDouble(i);

      // twist.twist.linear
      buf.putDouble(linearX);
      buf.putDouble(linearY);
      buf.putDouble(linearZ);
      // twist.twist.angular
      buf.putDouble(angularX);
      buf.putDouble(angularY);
      buf.putDouble(angularZ);
      // twist.covariance[36]
      for (int i = 0; i < 36; i++)
         buf.putDouble(-i);

      byte[] bytes = new byte[buf.position()];
      buf.flip();
      buf.get(bytes);
      return bytes;
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
      int rem = (buf.position() - 4) % alignment;
      if (rem != 0)
      {
         int pad = alignment - rem;
         for (int i = 0; i < pad; i++)
            buf.put((byte) 0);
      }
   }
}
