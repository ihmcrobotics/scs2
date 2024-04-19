package us.ihmc.scs2.session.mcap.specs.records;

public class MutableChannel implements Channel
{
   private int id;
   private int schemaId;
   private String topic;
   private String messageEncoding;
   private MetadataMap metadata;

   @Override
   public int id()
   {
      return id;
   }

   public void setId(int id)
   {
      this.id = id;
   }

   @Override
   public int schemaId()
   {
      return schemaId;
   }

   public void setSchemaId(int schemaId)
   {
      this.schemaId = schemaId;
   }

   @Override
   public String topic()
   {
      return topic;
   }

   public void setTopic(String topic)
   {
      this.topic = topic;
   }

   @Override
   public String messageEncoding()
   {
      return messageEncoding;
   }

   public void setMessageEncoding(String messageEncoding)
   {
      this.messageEncoding = messageEncoding;
   }

   @Override
   public MetadataMap metadata()
   {
      return metadata;
   }

   public void setMetadata(MetadataMap metadata)
   {
      this.metadata = metadata;
   }

   @Override
   public long getElementLength()
   {
      return
            // Id
            Short.BYTES
            // Schema ID
            + Short.BYTES
            // Topic
            + Integer.BYTES + topic.length()
            // Message Encoding
            + Integer.BYTES + messageEncoding.length()
            // Metadata
            + metadata.getElementLength();
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
