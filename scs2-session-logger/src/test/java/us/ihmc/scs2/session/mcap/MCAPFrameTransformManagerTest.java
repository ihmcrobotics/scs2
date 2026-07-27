package us.ihmc.scs2.session.mcap;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.session.mcap.MCAPFrameTransformManager.RawTransform;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Covers the tf2_msgs/TFMessage support added to {@link MCAPFrameTransformManager}: decoding a
 * {@code transforms[]} sequence of {@code geometry_msgs/TransformStamped} elements, which is a different wire shape
 * than the single-transform {@code foxglove::FrameTransform} schema the class originally only supported.
 */
public class MCAPFrameTransformManagerTest
{
   // Same schema text McapLogConverter.TF_SCHEMA emits for the /tf channel.
   private static final String TF_SCHEMA =
         "geometry_msgs/TransformStamped[] transforms\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/TransformStamped\n" +
         "std_msgs/Header header\n" +
         "string child_frame_id\n" +
         "geometry_msgs/Transform transform\n" +
         "\n================================================================================\n" +
         "MSG: std_msgs/Header\n" +
         "builtin_interfaces/Time stamp\n" +
         "string frame_id\n" +
         "\n================================================================================\n" +
         "MSG: builtin_interfaces/Time\n" +
         "int32 sec\n" +
         "uint32 nanosec\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Transform\n" +
         "geometry_msgs/Vector3 translation\n" +
         "geometry_msgs/Quaternion rotation\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Vector3\n" +
         "float64 x\n" +
         "float64 y\n" +
         "float64 z\n" +
         "\n================================================================================\n" +
         "MSG: geometry_msgs/Quaternion\n" +
         "float64 x\n" +
         "float64 y\n" +
         "float64 z\n" +
         "float64 w\n";

   @Test
   public void testIsTf2TFMessageSchemaName()
   {
      assertTrue(MCAPFrameTransformManager.isTf2TFMessageSchemaName("tf2_msgs/msg/TFMessage"));
      assertTrue(MCAPFrameTransformManager.isTf2TFMessageSchemaName("tf2_msgs/TFMessage"));
      assertTrue(MCAPFrameTransformManager.isTf2TFMessageSchemaName("tf2_msgs::msg::dds_::TFMessage_"));
      assertFalse(MCAPFrameTransformManager.isTf2TFMessageSchemaName("foxglove::FrameTransform"));
      assertFalse(MCAPFrameTransformManager.isTf2TFMessageSchemaName("sensor_msgs/msg/JointState"));
   }

   @Test
   public void testReadTransformStampedSequenceWithMultipleTransforms()
   {
      MCAPSchema loadedSchema = ROS2SchemaParser.loadSchema("tf2_msgs/msg/TFMessage", 1, TF_SCHEMA.getBytes(StandardCharsets.UTF_8));
      Map<String, MCAPSchema> subSchemaMap = loadedSchema.getSubSchemaMap();

      MCAPSchemaField transformsField = loadedSchema.getFields().get(0);
      assertEquals("transforms", transformsField.getName());
      MCAPSchema transformStampedSchema = subSchemaMap.get(transformsField.getType());

      MCAPFrameTransformManager.validateTransformStampedSchema(transformStampedSchema, subSchemaMap);

      RawTransform expectedFirst = new RawTransform("pelvis", "world", 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0);
      RawTransform expectedSecond = new RawTransform("map", "pelvis", 0.1, 0.2, 0.3, 0.9, -1.5, 0.25, 4.0);

      ByteBuffer buffer = encodeTfMessage(expectedFirst, expectedSecond);

      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(buffer, 0, buffer.position());

      int[] count = new int[1];
      RawTransform[] decoded = new RawTransform[2];
      cdr.read_sequence((elementIndex, elementCdr) ->
      {
         decoded[elementIndex] = MCAPFrameTransformManager.readTransformStamped(elementCdr, transformStampedSchema, subSchemaMap);
         count[0]++;
      });
      cdr.finalize(true);

      assertEquals(2, count[0]);
      assertRawTransformEquals(expectedFirst, decoded[0]);
      assertRawTransformEquals(expectedSecond, decoded[1]);
   }

   private static void assertRawTransformEquals(RawTransform expected, RawTransform actual)
   {
      assertEquals(expected.parentFrameName(), actual.parentFrameName());
      assertEquals(expected.childFrameName(), actual.childFrameName());
      assertEquals(expected.rx(), actual.rx(), 1e-12);
      assertEquals(expected.ry(), actual.ry(), 1e-12);
      assertEquals(expected.rz(), actual.rz(), 1e-12);
      assertEquals(expected.rw(), actual.rw(), 1e-12);
      assertEquals(expected.tx(), actual.tx(), 1e-12);
      assertEquals(expected.ty(), actual.ty(), 1e-12);
      assertEquals(expected.tz(), actual.tz(), 1e-12);
   }

   /**
    * Hand-encodes a {@code tf2_msgs/msg/TFMessage} CDR message carrying the given transforms, mirroring the
    * write-side logic in {@code McapLogConverter.buildTfMessage}.
    */
   private static ByteBuffer encodeTfMessage(RawTransform... transforms)
   {
      ByteBuffer buf = ByteBuffer.allocate(4096).order(ByteOrder.LITTLE_ENDIAN);
      // CDR encapsulation header: dummy byte, little-endian CDR marker, 2 option bytes.
      buf.put((byte) 0x00);
      buf.put((byte) 0x01);
      buf.put((byte) 0x00);
      buf.put((byte) 0x00);

      buf.putInt(transforms.length); // transforms[] sequence length

      for (RawTransform transform : transforms)
      {
         // header.stamp (int32 sec, uint32 nanosec)
         align(buf, 4);
         buf.putInt(0);
         buf.putInt(0);

         // header.frame_id
         writeString(buf, transform.parentFrameName());

         // child_frame_id
         writeString(buf, transform.childFrameName());

         // transform.translation
         align(buf, 8);
         buf.putDouble(transform.tx());
         buf.putDouble(transform.ty());
         buf.putDouble(transform.tz());

         // transform.rotation (x,y,z,w)
         buf.putDouble(transform.rx());
         buf.putDouble(transform.ry());
         buf.putDouble(transform.rz());
         buf.putDouble(transform.rw());
      }

      return buf;
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
