package us.ihmc.scs2.session.mcap;

import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.set.hash.TIntHashSet;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.session.mcap.MCAPBufferedChunk.ChunkBundle;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.MCAP;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.scs2.session.mcap.specs.records.Schema;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

public class MCAPFrameTransformManager
{
   private static final String FOXGLOVE_PREFIX = "foxglove::FrameTransform.";
   private static final String WORLD_FRAME_NAME = "world";
   private static final String FRAME_FIELD_TYPE = "string";
   private static final String PARENT_FRAME_FIELD_NAME = "parent_frame_id";
   private static final String CHILD_FRAME_FIELD_NAME = "child_frame_id";
   private static final String ROTATION_FIELD_NAME = "rotation";
   private static final String ROTATION_X_FIELD_NAME = "rotation.x";
   private static final String ROTATION_Y_FIELD_NAME = "rotation.y";
   private static final String ROTATION_Z_FIELD_NAME = "rotation.z";
   private static final String ROTATION_W_FIELD_NAME = "rotation.w";
   private static final String TRANSLATION_FIELD_NAME = "translation";
   private static final String TRANSLATION_X_FIELD_NAME = "translation.x";
   private static final String TRANSLATION_Y_FIELD_NAME = "translation.y";
   private static final String TRANSLATION_Z_FIELD_NAME = "translation.z";

   // tf2_msgs/TFMessage field names. Unlike the foxglove::FrameTransform schema above, these are read from the
   // *unflattened* schema, so nested struct field names are plain (e.g. "x", not "translation.x").
   private static final String TF2_TRANSFORMS_FIELD_NAME = "transforms";
   private static final String TF2_HEADER_FIELD_NAME = "header";
   private static final String TF2_HEADER_FRAME_ID_FIELD_NAME = "frame_id";
   private static final String TF2_TRANSFORM_FIELD_NAME = "transform";
   private static final String FIELD_X = "x";
   private static final String FIELD_Y = "y";
   private static final String FIELD_Z = "z";
   private static final String FIELD_W = "w";

   private enum FrameTransformSchemaKind
   {
      FOXGLOVE_FRAME_TRANSFORM, TF2_TF_MESSAGE
   }

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ReferenceFrame inertialFrame;
   private MCAPSchema frameTransformSchema;
   private FrameTransformSchemaKind frameTransformSchemaKind;
   /** Only set when {@link #frameTransformSchemaKind} is {@link FrameTransformSchemaKind#TF2_TF_MESSAGE}: the unflattened schema for one {@code geometry_msgs/TransformStamped} array element. */
   private MCAPSchema transformStampedSchema;
   /** Only set when {@link #frameTransformSchemaKind} is {@link FrameTransformSchemaKind#TF2_TF_MESSAGE}: the flat map of all sub-schema types referenced anywhere in the tf2_msgs/TFMessage schema (Header, Time, Transform, Vector3, Quaternion, ...). */
   private Map<String, MCAPSchema> tf2SubSchemaMap;
   private final List<YoFoxGloveFrameTransform> transformList = new ArrayList<>();
   private final Map<String, YoFoxGloveFrameTransform> rawNameToTransformMap = new LinkedHashMap<>();
   private final Map<String, YoFoxGloveFrameTransform> sanitizedNameToTransformMap = new LinkedHashMap<>();
   private final TIntHashSet channelIds = new TIntHashSet();
   /**
    * Sometimes, tfs are defined with a parent that doesn't exist, they are not yet attached to world.
    */
   private final Set<String> unattachedRootNames = new LinkedHashSet<>();

   private final YoGraphicGroupDefinition yoGraphicGroupDefinition = new YoGraphicGroupDefinition("FoxgloveFrameTransforms");
   private Schema mcapSchema;

   public MCAPFrameTransformManager(ReferenceFrame inertialFrame)
   {
      this.inertialFrame = inertialFrame;
   }

   public void initialize(MCAP mcap, MCAPBufferedChunk chunkBuffer) throws IOException
   {
      Schema foxgloveCandidate = null;
      Schema tf2Candidate = null;

      for (Record record : mcap.records())
      {
         if (record.op() != Opcode.SCHEMA)
            continue;

         Schema schema = (Schema) record.body();
         if (foxgloveCandidate == null && schema.name().equalsIgnoreCase("foxglove::FrameTransform"))
            foxgloveCandidate = schema;
         else if (tf2Candidate == null && isTf2TFMessageSchemaName(schema.name()))
            tf2Candidate = schema;
      }

      if (foxgloveCandidate != null)
      {
         frameTransformSchemaKind = FrameTransformSchemaKind.FOXGLOVE_FRAME_TRANSFORM;
         mcapSchema = foxgloveCandidate;
         if (tf2Candidate != null)
            LogTools.warn("Found both foxglove::FrameTransform and a tf2_msgs/TFMessage schema; using foxglove::FrameTransform and ignoring the tf2_msgs one.");
      }
      else if (tf2Candidate != null)
      {
         frameTransformSchemaKind = FrameTransformSchemaKind.TF2_TF_MESSAGE;
         mcapSchema = tf2Candidate;
      }
      else
      {
         LogTools.error("Could not find the schema for foxglove::FrameTransform or tf2_msgs/TFMessage");
         return;
      }

      MCAPSchema loadedSchema;
      if (mcapSchema.encoding().equalsIgnoreCase("ros2msg"))
      {
         loadedSchema = ROS2SchemaParser.loadSchema(mcapSchema);
      }
      else if (mcapSchema.encoding().equalsIgnoreCase("omgidl"))
      {
         loadedSchema = OMGIDLSchemaParser.loadSchema(mcapSchema);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported encoding: " + mcapSchema.encoding());
      }

      if (frameTransformSchemaKind == FrameTransformSchemaKind.FOXGLOVE_FRAME_TRANSFORM)
      {
         // Flatten the schema to make it easier to read.
         frameTransformSchema = loadedSchema.flattenSchema();
         for (String fieldName : Arrays.asList(PARENT_FRAME_FIELD_NAME,
                                               CHILD_FRAME_FIELD_NAME,
                                               ROTATION_FIELD_NAME,
                                               ROTATION_X_FIELD_NAME,
                                               ROTATION_Y_FIELD_NAME,
                                               ROTATION_Z_FIELD_NAME,
                                               ROTATION_W_FIELD_NAME,
                                               TRANSLATION_FIELD_NAME,
                                               TRANSLATION_X_FIELD_NAME,
                                               TRANSLATION_Y_FIELD_NAME,
                                               TRANSLATION_Z_FIELD_NAME))
         {
            if (frameTransformSchema.getFields().stream().noneMatch(field -> field.getName().equalsIgnoreCase(fieldName)))
               throw new RuntimeException("Could not find the field " + fieldName + " in the schema for foxglove::FrameTransform");
         }
      }
      else
      {
         // Keep the schema nested: the tf2_msgs/TFMessage reader walks the struct tree directly instead of relying on flattening,
         // since flattenSchema() does not know how to expand an unbounded sequence of structs (only fixed-size arrays).
         frameTransformSchema = loadedSchema;
         MCAPSchemaField transformsField = loadedSchema.getFields()
                                                        .stream()
                                                        .filter(field -> field.getName().equalsIgnoreCase(TF2_TRANSFORMS_FIELD_NAME))
                                                        .findFirst()
                                                        .orElseThrow(() -> new RuntimeException(
                                                              "Could not find the field '" + TF2_TRANSFORMS_FIELD_NAME
                                                              + "' in the schema for tf2_msgs/TFMessage"));
         tf2SubSchemaMap = loadedSchema.getSubSchemaMap();
         transformStampedSchema = tf2SubSchemaMap.get(transformsField.getType());
         if (transformStampedSchema == null)
            throw new RuntimeException("Could not find the sub-schema for type: " + transformsField.getType());
         validateTransformStampedSchema(transformStampedSchema, tf2SubSchemaMap);
      }

      TIntObjectHashMap<String> channelIdToTopicMap = new TIntObjectHashMap<>();
      for (Record record : mcap.records())
      {
         if (record.op() == Opcode.CHANNEL)
         {
            Channel channel = (Channel) record.body();
            if (channel.schemaId() == frameTransformSchema.getId())
            {
               channelIdToTopicMap.put(channel.id(), channel.topic());
            }
         }
      }
      channelIds.addAll(channelIdToTopicMap.keys());

      Map<String, BasicTransformInfo> allTransforms = new LinkedHashMap<>();

      for (Record record : mcap.records())
      {
         if (record.op() == Opcode.MESSAGE)
            processRecord(record, channelIdToTopicMap, allTransforms);
      }

      int numberOfTransforms = allTransforms.size();
      int numberOfChunkWithoutNewTransforms = 0;

      for (ChunkBundle chunkBundle : chunkBuffer.getChunkBundles())
      {
         chunkBundle.requestLoadChunkBundle(true, false, false);

         for (Record record : chunkBundle.getChunkRecords())
         {
            if (record.op() == Opcode.MESSAGE)
               processRecord(record, channelIdToTopicMap, allTransforms);
         }

         if (allTransforms.size() == numberOfTransforms)
            numberOfChunkWithoutNewTransforms++;
         else
            numberOfChunkWithoutNewTransforms = 0;

         if (numberOfChunkWithoutNewTransforms > 5)
            break;

         numberOfTransforms = allTransforms.size();
      }

      for (BasicTransformInfo transformInfo : allTransforms.values())
      {
         if (!allTransforms.containsKey(transformInfo.parentFrameName()) && !transformInfo.parentFrameName().equals(WORLD_FRAME_NAME))
         {
            unattachedRootNames.add(transformInfo.parentFrameName());
         }
      }

      if (!allTransforms.isEmpty())
      {
         LinkedList<BasicTransformInfo> ordered = sortTransforms(allTransforms);

         while (!ordered.isEmpty())
         {
            BasicTransformInfo basicTransformInfo = ordered.poll();
            YoFoxGloveFrameTransform transform = new YoFoxGloveFrameTransform(basicTransformInfo,
                                                                              rawNameToTransformMap.get(basicTransformInfo.parentFrameName()),
                                                                              inertialFrame,
                                                                              registry);
            yoGraphicGroupDefinition.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(transform.rawName,
                                                                                                        transform.poseToRoot,
                                                                                                        0.2,
                                                                                                        ColorDefinitions.SeaGreen()));
            rawNameToTransformMap.put(basicTransformInfo.childFrameName(), transform);
         }
         transformList.addAll(rawNameToTransformMap.values());
         for (YoFoxGloveFrameTransform transform : transformList)
         {
            sanitizedNameToTransformMap.put(transform.sanitizedName, transform);
         }
         yoGraphicGroupDefinition.setVisible(false);
      }
   }

   private void processRecord(Record record, TIntObjectHashMap<String> channelIdToTopicMap, Map<String, BasicTransformInfo> allTransforms)
   {
      Message message = (Message) record.body();
      String topic = channelIdToTopicMap.get(message.channelId());

      if (topic == null)
         return;

      for (BasicTransformInfo transformInfo : extractFromMessage(topic, message))
         allTransforms.put(transformInfo.childFrameName(), transformInfo);
   }

   private static LinkedList<BasicTransformInfo> sortTransforms(Map<String, BasicTransformInfo> allTransforms)
   {
      LinkedList<BasicTransformInfo> ordered = new LinkedList<>(allTransforms.values());
      ordered.sort((o1, o2) ->
                   {
                      int distanceToRoot1 = 0;
                      int distanceToRoot2 = 0;
                      while (o1 != null)
                      {
                         distanceToRoot1++;
                         o1 = allTransforms.get(o1.parentFrameName());
                      }
                      while (o2 != null)
                      {
                         distanceToRoot1++;
                         o2 = allTransforms.get(o2.parentFrameName());
                      }
                      return Integer.compare(distanceToRoot1, distanceToRoot2);
                   });
      return ordered;
   }

   public void update()
   {
      if (frameTransformSchemaKind == null)
         return;

      for (YoFoxGloveFrameTransform transform : transformList)
      {
         transform.update();
      }
   }

   private final CDRDeserializer cdr = new CDRDeserializer();

   /**
    * Tries to read the given message as a frame transform message.
    *
    * @param message the message to read.
    * @return {@code true} if the message was successfully read, {@code false} otherwise.
    */
   public boolean readMessage(Message message)
   {
      if (frameTransformSchemaKind == null)
         return false;

      if (!channelIds.contains(message.channelId()))
         return false;

      cdr.initialize(message.messageBuffer(), 0, message.dataLength());

      try
      {
         if (frameTransformSchemaKind == FrameTransformSchemaKind.TF2_TF_MESSAGE)
         {
            cdr.read_sequence((elementIndex, elementCdr) ->
            {
               RawTransform raw = readTransformStamped(elementCdr, transformStampedSchema, tf2SubSchemaMap);
               applyTransformUpdate(raw);
            });
         }
         else
         {
            List<? extends MCAPSchemaField> fields = frameTransformSchema.getFields();
            double rw = 1.0, rz = 0.0, ry = 0.0, rx = 0.0;
            double tz = 0.0, ty = 0.0, tx = 0.0;
            String parentFrameName = null;
            String childFrameName = null;

            for (int i = 0; i < fields.size(); i++)
            {
               MCAPSchemaField field = fields.get(i);
               if (field.isComplexType())
               {
                  if (field.getName().equalsIgnoreCase(ROTATION_FIELD_NAME))
                  {
                     MCAPSchemaField xField = fields.get(i + 1);
                     MCAPSchemaField yField = fields.get(i + 2);
                     MCAPSchemaField zField = fields.get(i + 3);
                     MCAPSchemaField wField = fields.get(i + 4);
                     if (!xField.getName().equalsIgnoreCase(ROTATION_X_FIELD_NAME))
                        throw new RuntimeException("Unexpected field name: " + xField.getName());
                     if (!yField.getName().equalsIgnoreCase(ROTATION_Y_FIELD_NAME))
                        throw new RuntimeException("Unexpected field name: " + yField.getName());
                     if (!zField.getName().equalsIgnoreCase(ROTATION_Z_FIELD_NAME))
                        throw new RuntimeException("Unexpected field name: " + zField.getName());
                     if (!wField.getName().equalsIgnoreCase(ROTATION_W_FIELD_NAME))
                        throw new RuntimeException("Unexpected field name: " + wField.getName());
                     rx = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(xField.getType()));
                     ry = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(yField.getType()));
                     rz = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(zField.getType()));
                     rw = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(wField.getType()));
                     i += 4;
                  }
                  else if (field.getName().equalsIgnoreCase(TRANSLATION_FIELD_NAME))
                  {
                     MCAPSchemaField xField = fields.get(i + 1);
                     MCAPSchemaField yField = fields.get(i + 2);
                     MCAPSchemaField zField = fields.get(i + 3);
                     if (!xField.getName().equalsIgnoreCase(TRANSLATION_X_FIELD_NAME))
                        throw new RuntimeException("Unexpected field name: " + xField.getName());
                     if (!yField.getName().equalsIgnoreCase(TRANSLATION_Y_FIELD_NAME))
                        throw new RuntimeException("Unexpected field name: " + yField.getName());
                     if (!zField.getName().equalsIgnoreCase(TRANSLATION_Z_FIELD_NAME))
                        throw new RuntimeException("Unexpected field name: " + zField.getName());
                     tx = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(xField.getType()));
                     ty = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(yField.getType()));
                     tz = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(zField.getType()));
                     i += 3;
                  }
               }
               else if (field.getType().equalsIgnoreCase(FRAME_FIELD_TYPE))
               {
                  if (field.getName().equalsIgnoreCase(PARENT_FRAME_FIELD_NAME))
                  {
                     parentFrameName = cdr.read_string();
                  }
                  else if (field.getName().equalsIgnoreCase(CHILD_FRAME_FIELD_NAME))
                  {
                     childFrameName = cdr.read_string();
                  }
               }
               else
               {
                  cdr.skipNext(CDRDeserializer.Type.parseType(field.getType()));
               }
            }

            applyTransformUpdate(new RawTransform(parentFrameName, childFrameName, rx, ry, rz, rw, tx, ty, tz));
         }
      }
      finally
      {
         cdr.finalize(true);
      }

      return true;
   }

   private void applyTransformUpdate(RawTransform raw)
   {
      YoFoxGloveFrameTransform transform = rawNameToTransformMap.get(raw.childFrameName());
      if (transform != null)
      {
         if (!Objects.equals(raw.parentFrameName(), transform.parentFrameName))
            LogTools.error("Unexpected parent frame name: " + raw.parentFrameName() + " for child frame: " + raw.childFrameName() + " expected: "
                           + transform.parentFrameName);

         transform.poseToParent.getOrientation().set(raw.rx(), raw.ry(), raw.rz(), raw.rw());
         transform.poseToParent.getPosition().set(raw.tx(), raw.ty(), raw.tz());
         transform.markPoseToRootAsDirty();
      }
      else
      {
         LogTools.error("Could not find transform for child frame: " + raw.childFrameName());
      }
   }

   public YoGraphicDefinition getYoGraphic()
   {
      return yoGraphicGroupDefinition;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public boolean hasMCAPFrameTransforms()
   {
      return frameTransformSchema != null;
   }

   public Schema getMCAPSchema()
   {
      return mcapSchema;
   }

   public MCAPSchema getFrameTransformSchema()
   {
      return frameTransformSchema;
   }

   public YoFoxGloveFrameTransform getTransformFromSanitizedName(String name)
   {
      return sanitizedNameToTransformMap.get(name);
   }

   private List<BasicTransformInfo> extractFromMessage(String topic, Message message)
   {
      if (frameTransformSchemaKind == FrameTransformSchemaKind.TF2_TF_MESSAGE)
      {
         List<BasicTransformInfo> infos = new ArrayList<>();
         CDRDeserializer cdr = new CDRDeserializer();
         cdr.initialize(message.messageBuffer(), 0, message.dataLength());
         cdr.read_sequence((elementIndex, elementCdr) ->
         {
            RawTransform raw = readTransformStamped(elementCdr, transformStampedSchema, tf2SubSchemaMap);
            infos.add(new BasicTransformInfo(topic,
                                             Objects.requireNonNull(raw.parentFrameName(),
                                                                    "Parent frame name is null for topic: " + topic + " and child: "
                                                                    + raw.childFrameName()),
                                             Objects.requireNonNull(raw.childFrameName(),
                                                                    "Child frame name is null for topic: " + topic + " and parent: "
                                                                    + raw.parentFrameName())));
         });
         cdr.finalize(true);
         return infos;
      }

      MCAPSchema flatSchema = frameTransformSchema;
      if (!flatSchema.isSchemaFlat())
         throw new IllegalArgumentException("The schema is not flat.");

      CDRDeserializer cdr = new CDRDeserializer();
      cdr.initialize(message.messageBuffer(), 0, message.dataLength());

      String parentFrameName = null;
      String childFrameName = null;

      for (MCAPSchemaField field : flatSchema.getFields())
      {
         if (field.isComplexType())
            continue;

         if (field.getType().equalsIgnoreCase(FRAME_FIELD_TYPE))
         {
            if (field.getName().equalsIgnoreCase(PARENT_FRAME_FIELD_NAME))
            {
               parentFrameName = cdr.read_string();
            }
            else if (field.getName().equalsIgnoreCase(CHILD_FRAME_FIELD_NAME))
            {
               childFrameName = cdr.read_string();
            }
         }
         else
         {
            cdr.skipNext(CDRDeserializer.Type.parseType(field.getType()));
         }
      }

      cdr.finalize(true);

      if (parentFrameName == null)
         throw new RuntimeException("Could not find the parent frame name for topic: " + topic);
      return Collections.singletonList(new BasicTransformInfo(topic,
                                                               Objects.requireNonNull(parentFrameName,
                                                                                      "Parent frame name is null for topic: " + topic + " and child: "
                                                                                      + childFrameName),
                                                               Objects.requireNonNull(childFrameName,
                                                                                      "Child frame name is null for topic: " + topic + " and parent: "
                                                                                      + parentFrameName)));
   }

   /**
    * Reads one {@code geometry_msgs/TransformStamped} element (as found in a {@code tf2_msgs/TFMessage.transforms} sequence)
    * from the current cursor position. The parent frame name is {@code header.frame_id} (ROS convention), not a
    * {@code parent_frame_id} field like {@code foxglove::FrameTransform} uses.
    */
   static RawTransform readTransformStamped(CDRDeserializer cdr, MCAPSchema transformStampedSchema, Map<String, MCAPSchema> subSchemaMap)
   {
      String parentFrameName = null;
      String childFrameName = null;
      double rw = 1.0, rx = 0.0, ry = 0.0, rz = 0.0;
      double tx = 0.0, ty = 0.0, tz = 0.0;

      for (MCAPSchemaField field : transformStampedSchema.getFields())
      {
         if (field.getName().equalsIgnoreCase(TF2_HEADER_FIELD_NAME))
         {
            MCAPSchema headerSchema = subSchemaMap.get(field.getType());
            for (MCAPSchemaField headerField : headerSchema.getFields())
            {
               if (headerField.getName().equalsIgnoreCase(TF2_HEADER_FRAME_ID_FIELD_NAME))
                  parentFrameName = cdr.read_string();
               else
                  skipField(cdr, headerField, subSchemaMap);
            }
         }
         else if (field.getName().equalsIgnoreCase(CHILD_FRAME_FIELD_NAME))
         {
            childFrameName = cdr.read_string();
         }
         else if (field.getName().equalsIgnoreCase(TF2_TRANSFORM_FIELD_NAME))
         {
            MCAPSchema transformSchema = subSchemaMap.get(field.getType());
            for (MCAPSchemaField transformField : transformSchema.getFields())
            {
               if (transformField.getName().equalsIgnoreCase(TRANSLATION_FIELD_NAME))
               {
                  MCAPSchema vector3Schema = subSchemaMap.get(transformField.getType());
                  for (MCAPSchemaField axisField : vector3Schema.getFields())
                  {
                     double value = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(axisField.getType()));
                     if (axisField.getName().equalsIgnoreCase(FIELD_X))
                        tx = value;
                     else if (axisField.getName().equalsIgnoreCase(FIELD_Y))
                        ty = value;
                     else if (axisField.getName().equalsIgnoreCase(FIELD_Z))
                        tz = value;
                  }
               }
               else if (transformField.getName().equalsIgnoreCase(ROTATION_FIELD_NAME))
               {
                  MCAPSchema quaternionSchema = subSchemaMap.get(transformField.getType());
                  for (MCAPSchemaField axisField : quaternionSchema.getFields())
                  {
                     double value = cdr.readTypeAsDouble(CDRDeserializer.Type.parseType(axisField.getType()));
                     if (axisField.getName().equalsIgnoreCase(FIELD_X))
                        rx = value;
                     else if (axisField.getName().equalsIgnoreCase(FIELD_Y))
                        ry = value;
                     else if (axisField.getName().equalsIgnoreCase(FIELD_Z))
                        rz = value;
                     else if (axisField.getName().equalsIgnoreCase(FIELD_W))
                        rw = value;
                  }
               }
               else
               {
                  skipField(cdr, transformField, subSchemaMap);
               }
            }
         }
         else
         {
            skipField(cdr, field, subSchemaMap);
         }
      }

      return new RawTransform(parentFrameName, childFrameName, rx, ry, rz, rw, tx, ty, tz);
   }

   /**
    * Generic fallback used to advance the CDR cursor past a field this reader doesn't otherwise care about, so that
    * later, recognized fields in the same struct stay correctly aligned. Complex fields are skipped by recursing into
    * their sub-schema; arrays/sequences of complex types are not supported here since none appear in the tf2_msgs
    * message tree this reader targets.
    */
   private static void skipField(CDRDeserializer cdr, MCAPSchemaField field, Map<String, MCAPSchema> subSchemaMap)
   {
      if (!field.isComplexType())
      {
         cdr.skipNext(CDRDeserializer.Type.parseType(field.getType()));
         return;
      }

      if (field.isArray() || field.isVector())
         throw new UnsupportedOperationException("Skipping arrays/sequences of complex fields is not supported: " + field.getName());

      MCAPSchema subSchema = subSchemaMap.get(field.getType());
      if (subSchema == null)
         throw new IllegalStateException("Could not find a schema for the type: " + field.getType());
      for (MCAPSchemaField subField : subSchema.getFields())
         skipField(cdr, subField, subSchemaMap);
   }

   /**
    * Validates that the given {@code geometry_msgs/TransformStamped} schema has the fields
    * {@link #readTransformStamped} expects, failing fast with a clear message instead of a confusing CDR misalignment
    * error later on.
    */
   static void validateTransformStampedSchema(MCAPSchema transformStampedSchema, Map<String, MCAPSchema> subSchemaMap)
   {
      MCAPSchemaField headerField = requireField(transformStampedSchema, TF2_HEADER_FIELD_NAME);
      MCAPSchema headerSchema = subSchemaMap.get(headerField.getType());
      if (headerSchema == null)
         throw new RuntimeException("Could not find the sub-schema for type: " + headerField.getType());
      requireField(headerSchema, TF2_HEADER_FRAME_ID_FIELD_NAME);

      requireField(transformStampedSchema, CHILD_FRAME_FIELD_NAME);

      MCAPSchemaField transformField = requireField(transformStampedSchema, TF2_TRANSFORM_FIELD_NAME);
      MCAPSchema transformSchema = subSchemaMap.get(transformField.getType());
      if (transformSchema == null)
         throw new RuntimeException("Could not find the sub-schema for type: " + transformField.getType());

      MCAPSchemaField translationField = requireField(transformSchema, TRANSLATION_FIELD_NAME);
      MCAPSchema vector3Schema = subSchemaMap.get(translationField.getType());
      if (vector3Schema == null)
         throw new RuntimeException("Could not find the sub-schema for type: " + translationField.getType());
      requireField(vector3Schema, FIELD_X);
      requireField(vector3Schema, FIELD_Y);
      requireField(vector3Schema, FIELD_Z);

      MCAPSchemaField rotationField = requireField(transformSchema, ROTATION_FIELD_NAME);
      MCAPSchema quaternionSchema = subSchemaMap.get(rotationField.getType());
      if (quaternionSchema == null)
         throw new RuntimeException("Could not find the sub-schema for type: " + rotationField.getType());
      requireField(quaternionSchema, FIELD_X);
      requireField(quaternionSchema, FIELD_Y);
      requireField(quaternionSchema, FIELD_Z);
      requireField(quaternionSchema, FIELD_W);
   }

   private static MCAPSchemaField requireField(MCAPSchema schema, String fieldName)
   {
      return schema.getFields()
                   .stream()
                   .filter(field -> field.getName().equalsIgnoreCase(fieldName))
                   .findFirst()
                   .orElseThrow(() -> new RuntimeException(
                         "Could not find the field '" + fieldName + "' in the schema for tf2_msgs/TFMessage (schema: " + schema.getName() + ")"));
   }

   /**
    * Matches {@code tf2_msgs/msg/TFMessage} (ROS2), {@code tf2_msgs/TFMessage} (ROS1), and OMGIDL/CycloneDDS-mangled
    * variants such as {@code tf2_msgs::msg::dds_::TFMessage_} that real {@code ros2 bag record} MCAP files use.
    */
   static boolean isTf2TFMessageSchemaName(String rawSchemaName)
   {
      String normalized = rawSchemaName.toLowerCase().replaceAll("[^a-z0-9]", "");
      return normalized.contains("tf2msgs") && normalized.contains("tfmessage");
   }

   private record BasicTransformInfo(String topic, String parentFrameName, String childFrameName)
   {

   }

   record RawTransform(String parentFrameName, String childFrameName, double rx, double ry, double rz, double rw, double tx, double ty, double tz)
   {

   }

   public static class YoFoxGloveFrameTransform
   {
      private final String parentFrameName;
      private final String rawName;
      private final String sanitizedName;
      private YoFoxGloveFrameTransform parent;
      private final List<YoFoxGloveFrameTransform> children;
      private final YoPose3D poseToParent;
      private final YoFramePose3D poseToRoot;

      private boolean isPoseToRootDirty = true;

      private YoFoxGloveFrameTransform(BasicTransformInfo info, YoFoxGloveFrameTransform parent, ReferenceFrame inertialFrame, YoRegistry registry)
      {
         parentFrameName = info.parentFrameName();
         rawName = info.childFrameName();
         sanitizedName = sanitizeName(rawName);
         children = new ArrayList<>();
         String namePrefix = sanitizedName;
         String worldNamePrefix = sanitizeName(namePrefix + "_world");
         poseToParent = new YoPose3D(namePrefix, registry);
         if (parent == null)
         {
            YoPoint3D yoPosition = poseToParent.getPosition();
            YoQuaternion yoOrientation = poseToParent.getOrientation();
            poseToRoot = new YoFramePose3D(new YoFramePoint3D(yoPosition.getYoX(), yoPosition.getYoY(), yoPosition.getYoZ(), inertialFrame),
                                           new YoFrameQuaternion(yoOrientation.getYoQx(),
                                                                 yoOrientation.getYoQy(),
                                                                 yoOrientation.getYoQz(),
                                                                 yoOrientation.getYoQs(),
                                                                 inertialFrame));
         }
         else
         {
            poseToRoot = new YoFramePose3D(worldNamePrefix, inertialFrame, registry);
         }
         setParent(parent);
      }

      private static String sanitizeName(String name)
      {
         name = name.replace('.', '_').replaceAll("_+", "_");
         return name.startsWith("_") ? name.substring(1) : name;
      }

      public void setParent(YoFoxGloveFrameTransform parent)
      {
         if (this.parent != null)
            throw new IllegalStateException("Parent already set.");
         this.parent = parent;
         if (parent != null)
         {
            if (!parent.rawName.equals(parentFrameName))
               throw new IllegalArgumentException("Unexpected parent frame name: " + parent.rawName + " expected: " + parentFrameName);
            parent.addChild(this);
         }
      }

      public void addChild(YoFoxGloveFrameTransform child)
      {
         children.add(child);
      }

      public void markPoseToRootAsDirty()
      {
         isPoseToRootDirty = true;

         for (YoFoxGloveFrameTransform child : children)
         {
            child.markPoseToRootAsDirty();
         }
      }

      public void update()
      {
         if (parent != null && isPoseToRootDirty)
         {
            if (parent.isPoseToRootDirty)
               parent.update();
            poseToRoot.set(parent.poseToRoot);
            poseToRoot.multiply(poseToParent);
         }
         isPoseToRootDirty = false;
      }

      public String getRawName()
      {
         return rawName;
      }

      public String getSanitizedName()
      {
         return sanitizedName;
      }

      public YoFoxGloveFrameTransform getParent()
      {
         return parent;
      }

      public RigidBodyTransformReadOnly getTransformToParent()
      {
         return poseToParent;
      }

      public RigidBodyTransformReadOnly getTransformToRoot()
      {
         return poseToRoot;
      }
   }
}
