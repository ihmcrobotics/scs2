package us.ihmc.scs2.session.mcap.specs.records;

public class MutableMetadataIndex implements MetadataIndex
{
   private long metadataOffset;
   private long metadataLength;
   private String name;
   private Record metadata;

   @Override
   public Record metadata()
   {
      return metadata;
   }

   public void setMetadata(Record metadata)
   {
      this.metadata = metadata;
      Metadata metadataBody = metadata.body();
      metadataLength = metadata.getElementLength();
      name = metadataBody.name();
   }

   @Override
   public long metadataOffset()
   {
      return metadataOffset;
   }

   public void setMetadataOffset(long metadataOffset)
   {
      this.metadataOffset = metadataOffset;
   }

   @Override
   public long metadataLength()
   {
      return metadataLength;
   }

   public void setMetadataLength(long metadataLength)
   {
      this.metadataLength = metadataLength;
   }

   @Override
   public String name()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof MetadataIndex other && MetadataIndex.super.equals(other);
   }
}
