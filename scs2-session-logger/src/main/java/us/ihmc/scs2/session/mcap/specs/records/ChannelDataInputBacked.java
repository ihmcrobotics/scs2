package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.util.List;

class ChannelDataInputBacked implements Channel
{
   private final MCAPDataInput dataInput;
   private final long elementLength;
   private final int id;
   private final int schemaId;
   private final String topic;
   private final String messageEncoding;
   private WeakReference<List<StringPair>> metadataRef;
   private final long metadataOffset;
   private final long metadataLength;

   public ChannelDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;
      this.elementLength = elementLength;

      dataInput.position(elementPosition);
      id = dataInput.getUnsignedShort();
      schemaId = dataInput.getUnsignedShort();
      topic = dataInput.getString();
      messageEncoding = dataInput.getString();
      metadataLength = dataInput.getUnsignedInt();
      metadataOffset = dataInput.position();
   }

   @Override
   public long getElementLength()
   {
      return elementLength;
   }

   @Override
   public int id()
   {
      return id;
   }

   @Override
   public int schemaId()
   {
      return schemaId;
   }

   @Override
   public String topic()
   {
      return topic;
   }

   @Override
   public String messageEncoding()
   {
      return messageEncoding;
   }

   @Override
   public List<StringPair> metadata()
   {
      List<StringPair> metadata = metadataRef == null ? null : metadataRef.get();

      if (metadata == null)
      {
         metadata = MCAP.parseList(dataInput, StringPair::new, metadataOffset, metadataLength);
         metadataRef = new WeakReference<>(metadata);
      }

      return metadata;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Channel other && Channel.super.equals(other);
   }
}
