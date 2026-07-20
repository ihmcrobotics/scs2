package us.ihmc.scs2.session.mcap;

import com.google.protobuf.Descriptors.Descriptor;

import java.util.List;
import java.util.Map;

/**
 * A {@link MCAPSchema} built from a MCAP schema whose encoding is {@code protobuf}.
 * <p>
 * In addition to the usual struct representation (used for schema bookkeeping/debug display), this carries the
 * {@link Descriptor} that was resolved while parsing the schema's {@code FileDescriptorSet}, needed to build a
 * {@link YoMCAPProtobufMessage} that can actually decode messages on the channel via
 * {@code com.google.protobuf.DynamicMessage}.
 * </p>
 */
public class MCAPProtobufSchema extends MCAPSchema
{
   private final Descriptor descriptor;

   MCAPProtobufSchema(String name, int id, List<MCAPSchemaField> fields, Map<String, MCAPSchema> subSchemaMap, Descriptor descriptor)
   {
      super(name, id, fields, subSchemaMap);
      this.descriptor = descriptor;
   }

   public Descriptor getDescriptor()
   {
      return descriptor;
   }
}
