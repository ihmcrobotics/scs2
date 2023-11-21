package us.ihmc.scs2.session.mcap;

import java.util.List;

public interface MCAPSchema
{
   //TODO: (AM) complete interface implementation
   int getId();

   String getName();

   List<? extends MCAPSchemaField> getFields();

   <T extends MCAPSchema> T flattenSchema();

   boolean isSchemaFlat();

   interface MCAPSchemaField
   {
      //TODO: (AM) complete interface implementation
      String getName();
      String getType();
      <T extends MCAPSchemaField> T getParent();
      boolean isArray();
      boolean isComplexType();
      boolean isVector();
   }
}
