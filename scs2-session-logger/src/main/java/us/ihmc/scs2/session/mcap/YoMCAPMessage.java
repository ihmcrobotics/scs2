package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.MCAPSchema.MCAPSchemaField;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.function.Consumer;

public interface YoMCAPMessage
{
   //TODO: (AM) complete interface implementation

   <T extends MCAPSchema> T getSchema();

   YoRegistry getRegistry();

   <T extends YoMCAPMessage> T newMessage(int channelId, MCAPSchema schema);

   static Consumer<CDRDeserializer> createYoVariable(MCAPSchemaField field, YoRegistry registry)
   {
      return null;
   }

   static Consumer<CDRDeserializer> createYoVariableArray(MCAPSchemaField field, YoRegistry registry)
   {
      return null;
   }

   void readMessage(MCAP.Message message);
}
