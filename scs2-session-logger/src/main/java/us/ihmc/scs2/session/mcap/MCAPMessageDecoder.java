package us.ihmc.scs2.session.mcap;

import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Common contract for decoding MCAP messages of a given channel into {@code YoVariable}s, regardless of the
 * message encoding (e.g. {@code cdr} vs {@code protobuf}).
 *
 * @see YoMCAPMessage for the {@code cdr} implementation.
 * @see YoMCAPProtobufMessage for the {@code protobuf} implementation.
 */
public interface MCAPMessageDecoder
{
   /**
    * Decodes the given message and updates the backing {@code YoVariable}s.
    *
    * @param message the message to decode, must belong to {@link #getChannelId()}.
    */
   void readMessage(Message message);

   /**
    * The registry holding the {@code YoVariable}s populated by {@link #readMessage(Message)}.
    *
    * @return the registry.
    */
   YoRegistry getRegistry();

   /**
    * The schema this decoder was built from.
    *
    * @return the schema.
    */
   MCAPSchema getSchema();

   /**
    * The ID of the channel this decoder is associated with.
    *
    * @return the channel ID.
    */
   int getChannelId();
}
