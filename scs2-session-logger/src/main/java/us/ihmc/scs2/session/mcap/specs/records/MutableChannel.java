package us.ihmc.scs2.session.mcap.specs.records;

import java.util.List;

public class MutableChannel implements Channel
{
   private int id;
   private int schemaId;
   private String topic;
   private String messageEncoding;
   private List<StringPair> metadata;

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
   public List<StringPair> metadata()
   {
      return metadata;
   }

   public void setMetadata(List<StringPair> metadata)
   {
      this.metadata = metadata;
   }

   @Override
   public long getElementLength()
   {
      int metadataLength = Integer.BYTES;
      for (int i = 0; i < metadata.size(); i++)
      {
         metadataLength += Integer.BYTES + metadata.get(i).key().length() + Integer.BYTES + metadata.get(i).value().length();
      }
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
            + metadataLength;
   }
}
