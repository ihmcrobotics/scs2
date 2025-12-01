package us.ihmc.scs2.session.mcap;

import us.ihmc.jros2.parser.field.InterfaceField;
import us.ihmc.jros2.parser.field.InterfaceFieldParsingException;
import us.ihmc.jros2.parser.msgdeps.MsgDepsContext;
import us.ihmc.jros2.parser.msgdeps.MsgDepsParser;
import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.scs2.session.mcap.specs.records.Schema;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Class used to represent a Java interpreter of a MCAP schema which encoding is "ros2msg".
 * This schema resembles much of ROS2 messages.
 */
public class ROS2SchemaParser
{
   private static MCAPSchemaField convertField(InterfaceField field)
   {
      MCAPSchemaField mcapSchemaField = new MCAPSchemaField();
      mcapSchemaField.setName(field.getName());
      mcapSchemaField.setType(field.getType());
      mcapSchemaField.setArray(field.isArray());
      mcapSchemaField.setVector(field.isUpperBounded()); // TODO: Check
      mcapSchemaField.setMaxLength(field.getLength());
      mcapSchemaField.setComplexType(!field.isBuiltinType());
      mcapSchemaField.setDefaultValue(field.getDefaultValue());
      return mcapSchemaField;
   }

   public static MCAPSchema loadSchema(Schema schema)
   {
      String packageResourceName = schema.name().replace("/msg", "");
      String msgContent = new String(schema.data().array());

      MsgDepsContext context;

      try
      {
         context = MsgDepsParser.parseMsgDeps(msgContent, packageResourceName);
      }
      catch (InterfaceFieldParsingException e)
      {
         throw new RuntimeException(e);
      }

      List<MCAPSchemaField> schemaFields = context.getFieldList().stream().map(ROS2SchemaParser::convertField).toList();
      Map<String, MCAPSchema> subSchemaMap = new LinkedHashMap<>();
      context.getDependencies().forEach((type, dependencyContext) ->
                                        {
                                           List<MCAPSchemaField> dependencySchemaFields = dependencyContext.getFieldList()
                                                                                                           .stream()
                                                                                                           .map(ROS2SchemaParser::convertField)
                                                                                                           .toList();
                                           MCAPSchema dependencySchema = new MCAPSchema(type, schema.id(), dependencySchemaFields);
                                           subSchemaMap.put(type, dependencySchema);
                                        });

      return new MCAPSchema(schema.name(), schema.id(), schemaFields, subSchemaMap);
   }
}
