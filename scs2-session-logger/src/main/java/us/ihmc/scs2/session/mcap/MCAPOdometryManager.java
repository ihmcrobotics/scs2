package us.ihmc.scs2.session.mcap;

import gnu.trove.set.hash.TIntHashSet;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.util.Map;

/**
 * Resolves a floating-base joint's exact twist from a standard {@code nav_msgs/msg/Odometry} channel, when the
 * file has one. Unlike {@code /tf} (pose only), {@code nav_msgs/Odometry} carries a {@code twist} field directly,
 * so no approximation (e.g. finite-differencing consecutive poses) is needed.
 * <p>
 * {@code nav_msgs/Odometry.pose} (a {@code PoseWithCovariance}) itself has a field named {@code pose} (a
 * {@code Pose}) - the standard message nests a field with the same name as its container. That collides with
 * {@link YoMCAPMessage}'s generic decoder, which builds one {@link us.ihmc.yoVariables.registry.YoRegistry} per
 * complex field name: {@code odom.pose.pose} is rejected by SCS2's namespace sanity check. This isn't specific to
 * this converter's output - any real {@code nav_msgs/Odometry} file would hit the same collision. So, like
 * {@link MCAPFrameTransformManager} already does for {@code /tf}, this channel is excluded from generic decoding
 * entirely and hand-parsed here instead (see {@link MCAPLogFileReader}'s exclusion checks in {@code loadSchemas()}/
 * {@code loadChannels()}, mirroring {@code frameTransformManager}'s).
 */
public class MCAPOdometryManager
{
   private static final String TWIST_FIELD_NAME = "twist";
   private static final String LINEAR_FIELD_NAME = "linear";
   private static final String ANGULAR_FIELD_NAME = "angular";
   private static final String FIELD_X = "x";
   private static final String FIELD_Y = "y";
   private static final String FIELD_Z = "z";

   private final TIntHashSet channelIds = new TIntHashSet();
   private final CDRDeserializer cdr = new CDRDeserializer();

   private Schema odometrySchema;
   private MCAPSchema loadedOdometrySchema;
   private Map<String, MCAPSchema> subSchemaMap;

   private boolean hasData = false;
   private double angularX, angularY, angularZ;
   private double linearX, linearY, linearZ;

   public void initialize(MCAP mcap) throws java.io.IOException
   {
      Schema matchedSchema = null;
      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.SCHEMA)
            continue;
         Schema schema = (Schema) record.body();
         if (isOdometrySchemaName(schema.name()))
         {
            matchedSchema = schema;
            break;
         }
      }

      if (matchedSchema == null)
         return; // No standard Odometry channel in this file; the caller falls back to another velocity source.

      MCAPSchema loadedSchema;
      if (matchedSchema.encoding().equalsIgnoreCase("ros2msg"))
         loadedSchema = ROS2SchemaParser.loadSchema(matchedSchema);
      else if (matchedSchema.encoding().equalsIgnoreCase("omgidl"))
         loadedSchema = OMGIDLSchemaParser.loadSchema(matchedSchema);
      else
         return;

      odometrySchema = matchedSchema;
      loadedOdometrySchema = loadedSchema;
      subSchemaMap = loadedSchema.getSubSchemaMap();

      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.CHANNEL)
            continue;
         Channel channel = (Channel) record.body();
         if (channel.schemaId() == matchedSchema.id())
            channelIds.add(channel.id());
      }
   }

   public boolean hasOdometrySchema()
   {
      return odometrySchema != null;
   }

   public int getOdometrySchemaId()
   {
      return odometrySchema.id();
   }

   /**
    * Tries to read the given message as an Odometry message, caching its twist if so.
    *
    * @return {@code true} if the message was on a recognized Odometry channel.
    */
   public boolean readMessage(Message message)
   {
      if (odometrySchema == null || !channelIds.contains(message.channelId()))
         return false;

      cdr.initialize(message.messageBuffer(), 0, message.dataLength());
      try
      {
         OdometryTwist twist = readTwist(cdr, loadedOdometrySchema, subSchemaMap);
         angularX = twist.angularX();
         angularY = twist.angularY();
         angularZ = twist.angularZ();
         linearX = twist.linearX();
         linearY = twist.linearY();
         linearZ = twist.linearZ();
         hasData = true;
      }
      finally
      {
         cdr.finalize(true);
      }
      return true;
   }

   /**
    * @return {@code true} if this file has a standard Odometry channel and at least one message has been read from
    *       it, i.e. if {@link #getAngularVelocityX()} etc. return real data rather than stale defaults.
    */
   public boolean hasTwist()
   {
      return hasData;
   }

   public double getAngularVelocityX()
   {
      return angularX;
   }

   public double getAngularVelocityY()
   {
      return angularY;
   }

   public double getAngularVelocityZ()
   {
      return angularZ;
   }

   public double getLinearVelocityX()
   {
      return linearX;
   }

   public double getLinearVelocityY()
   {
      return linearY;
   }

   public double getLinearVelocityZ()
   {
      return linearZ;
   }

   /**
    * Reads one {@code nav_msgs/Odometry} message from the current cursor position, extracting just its
    * {@code twist.twist.linear}/{@code .angular} fields and skipping everything else ({@code header},
    * {@code child_frame_id}, {@code pose}, and both {@code covariance} arrays) via {@link MCAPFrameTransformManager#skipField}.
    */
   static OdometryTwist readTwist(CDRDeserializer cdr, MCAPSchema odometrySchema, Map<String, MCAPSchema> subSchemaMap)
   {
      double angularX = 0.0, angularY = 0.0, angularZ = 0.0;
      double linearX = 0.0, linearY = 0.0, linearZ = 0.0;

      for (MCAPSchemaField field : odometrySchema.getFields())
      {
         if (!field.getName().equalsIgnoreCase(TWIST_FIELD_NAME))
         {
            MCAPFrameTransformManager.skipField(cdr, field, subSchemaMap);
            continue;
         }

         // geometry_msgs/TwistWithCovariance: a field also named "twist" (geometry_msgs/Twist), plus a covariance[36].
         MCAPSchema twistWithCovarianceSchema = subSchemaMap.get(field.getType());
         for (MCAPSchemaField twistWithCovarianceField : twistWithCovarianceSchema.getFields())
         {
            if (!twistWithCovarianceField.getName().equalsIgnoreCase(TWIST_FIELD_NAME))
            {
               MCAPFrameTransformManager.skipField(cdr, twistWithCovarianceField, subSchemaMap);
               continue;
            }

            MCAPSchema twistSchema = subSchemaMap.get(twistWithCovarianceField.getType());
            for (MCAPSchemaField twistField : twistSchema.getFields())
            {
               if (twistField.getName().equalsIgnoreCase(LINEAR_FIELD_NAME))
               {
                  double[] xyz = readVector3(cdr, twistField, subSchemaMap);
                  linearX = xyz[0];
                  linearY = xyz[1];
                  linearZ = xyz[2];
               }
               else if (twistField.getName().equalsIgnoreCase(ANGULAR_FIELD_NAME))
               {
                  double[] xyz = readVector3(cdr, twistField, subSchemaMap);
                  angularX = xyz[0];
                  angularY = xyz[1];
                  angularZ = xyz[2];
               }
               else
               {
                  MCAPFrameTransformManager.skipField(cdr, twistField, subSchemaMap);
               }
            }
         }
      }

      return new OdometryTwist(angularX, angularY, angularZ, linearX, linearY, linearZ);
   }

   private static double[] readVector3(CDRDeserializer cdr, MCAPSchemaField field, Map<String, MCAPSchema> subSchemaMap)
   {
      double[] xyz = new double[3];
      MCAPSchema vectorSchema = subSchemaMap.get(field.getType());
      for (MCAPSchemaField axisField : vectorSchema.getFields())
      {
         double value = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(axisField.getType()));
         if (axisField.getName().equalsIgnoreCase(FIELD_X))
            xyz[0] = value;
         else if (axisField.getName().equalsIgnoreCase(FIELD_Y))
            xyz[1] = value;
         else if (axisField.getName().equalsIgnoreCase(FIELD_Z))
            xyz[2] = value;
      }
      return xyz;
   }

   /**
    * Matches {@code nav_msgs/msg/Odometry} (ROS2), {@code nav_msgs/Odometry} (ROS1), and OMGIDL/CycloneDDS-mangled
    * variants such as {@code nav_msgs::msg::dds_::Odometry_}.
    */
   static boolean isOdometrySchemaName(String rawSchemaName)
   {
      String normalized = rawSchemaName.toLowerCase().replaceAll("[^a-z0-9]", "");
      return normalized.contains("navmsgs") && normalized.contains("odometry");
   }

   record OdometryTwist(double angularX, double angularY, double angularZ, double linearX, double linearY, double linearZ)
   {
   }
}
