package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAPBufferedChunk.ChunkBundle;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Schema;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Resolves per-joint velocity from a standard {@code sensor_msgs/msg/JointState} channel — the
 * near-universal ROS convention (MoveIt, ros_control, joint_state_publisher, etc. all publish
 * one) for reporting joint {@code name[]}/{@code position[]}/{@code velocity[]}/{@code effort[]}.
 * <p>
 * That channel isn't excluded from {@link MCAPLogFileReader}'s generic per-channel decoding
 * (unlike {@code /tf}), so {@link YoMCAPMessage} already builds and live-updates
 * {@code velocity[i]} as an ordinary, ever-updating {@code YoDouble} — no new per-tick decoding
 * is needed for the numbers themselves. The only piece the generic decoder can't recover is which
 * array index belongs to which joint: {@code name[]} is a {@code string[]} field, and
 * {@link YoMCAPMessage}'s generic decoder deliberately does not retain string values (it only
 * reads them to keep the CDR cursor advancing). This class recovers that index mapping once, from
 * the first {@code JointState} message found in the file.
 */
public class MCAPJointStateManager
{
   private static final String NAME_FIELD_NAME = "name";
   private static final String VELOCITY_FIELD_NAME = "velocity";

   private final Map<String, Integer> jointNameToArrayIndex = new HashMap<>();
   private MCAPMessageDecoder jointStateMessage;

   public void initialize(MCAP mcap, MCAPBufferedChunk chunkBuffer, TIntObjectHashMap<MCAPMessageDecoder> yoMessageMap) throws IOException
   {
      Schema matchedSchema = null;
      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.SCHEMA)
            continue;
         Schema schema = (Schema) record.body();
         if (isJointStateSchemaName(schema.name()))
         {
            matchedSchema = schema;
            break;
         }
      }

      if (matchedSchema == null)
         return; // No standard JointState channel in this file; OneDoF velocity will simply stay unavailable.

      MCAPSchema loadedSchema;
      if (matchedSchema.encoding().equalsIgnoreCase("ros2msg"))
         loadedSchema = ROS2SchemaParser.loadSchema(matchedSchema);
      else if (matchedSchema.encoding().equalsIgnoreCase("omgidl"))
         loadedSchema = OMGIDLSchemaParser.loadSchema(matchedSchema);
      else
         return;

      int channelId = -1;
      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.CHANNEL)
            continue;
         Channel channel = (Channel) record.body();
         if (channel.schemaId() == matchedSchema.id())
         {
            channelId = channel.id();
            break;
         }
      }

      if (channelId == -1)
         return;

      jointStateMessage = yoMessageMap.get(channelId);
      if (jointStateMessage == null)
         return;

      Message firstMessage = findFirstMessage(mcap, chunkBuffer, channelId);
      if (firstMessage == null)
      {
         LogTools.warn("Could not find any sensor_msgs/JointState message to resolve joint name ordering; OneDoF joint velocities will be unavailable.");
         jointStateMessage = null;
         return;
      }

      List<String> names = readNameArray(loadedSchema, firstMessage);
      for (int i = 0; i < names.size(); i++)
         jointNameToArrayIndex.put(names.get(i), i);
   }

   /**
    * @return the live-updating {@code velocity[i]} {@code YoDouble} for the given joint name, or {@code null} if this
    *       file has no standard JointState channel, or no field matched that joint name.
    */
   public YoDouble getVelocity(String jointName)
   {
      if (jointStateMessage == null)
         return null;
      Integer index = jointNameToArrayIndex.get(jointName);
      if (index == null)
         return null;
      YoVariable variable = jointStateMessage.getRegistry().findVariable(VELOCITY_FIELD_NAME + "[" + index + "]");
      return variable instanceof YoDouble ? (YoDouble) variable : null;
   }

   /**
    * Finds the first recorded {@link Message} on {@code channelId}, checking top-level records before falling back to
    * scanning chunk bundles. Shared with {@link MCAPLogFileReader}, which uses it to resolve a sample message for
    * protobuf {@code map} fields (see {@link YoMCAPProtobufMessage}).
    */
   static Message findFirstMessage(MCAP mcap, MCAPBufferedChunk chunkBuffer, int channelId) throws IOException
   {
      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.MESSAGE)
            continue;
         Message message = (Message) record.body();
         if (message.channelId() == channelId)
            return message;
      }

      for (ChunkBundle bundle : chunkBuffer.getChunkBundles())
      {
         bundle.requestLoadChunkBundle(true, false, false);
         for (Record record : bundle.getChunkRecords())
         {
            if (record.op() != Opcode.MESSAGE)
               continue;
            Message message = (Message) record.body();
            if (message.channelId() == channelId)
               return message;
         }
      }

      return null;
   }

   /**
    * Reads just the {@code name[]} field of one {@code sensor_msgs/JointState} message, skipping over
    * {@code header} (and stopping immediately after {@code name} — {@code position}/{@code velocity}/{@code effort}
    * are already handled generically elsewhere, so there's no need to keep decoding).
    */
   static List<String> readNameArray(MCAPSchema schema, Message message)
   {
      List<String> names = new ArrayList<>();
      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(message.messageBuffer(), 0, message.dataLength());

      try
      {
         for (MCAPSchemaField field : schema.getFields())
         {
            if (field.getName().equalsIgnoreCase(NAME_FIELD_NAME))
            {
               cdr.read_sequence((elementIndex, elementCdr) -> names.add(elementCdr.read_string()));
               break;
            }
            else
            {
               MCAPFrameTransformManager.skipField(cdr, field, schema.getSubSchemaMap());
            }
         }
      }
      finally
      {
         cdr.finalize(true);
      }

      return names;
   }

   /**
    * Matches {@code sensor_msgs/msg/JointState} (ROS2), {@code sensor_msgs/JointState} (ROS1), and
    * OMGIDL/CycloneDDS-mangled variants such as {@code sensor_msgs::msg::dds_::JointState_}.
    */
   static boolean isJointStateSchemaName(String rawSchemaName)
   {
      String normalized = rawSchemaName.toLowerCase().replaceAll("[^a-z0-9]", "");
      return normalized.contains("sensormsgs") && normalized.contains("jointstate");
   }
}
