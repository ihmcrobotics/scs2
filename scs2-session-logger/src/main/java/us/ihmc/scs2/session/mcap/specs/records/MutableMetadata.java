package us.ihmc.scs2.session.mcap.specs.records;

public class MutableMetadata implements Metadata
{
   private String name;
   private MetadataMap metadata;

   public MutableMetadata()
   {
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
   public MetadataMap metadata()
   {
      return metadata;
   }

   public void setMetadata(MetadataMap metadata)
   {
      this.metadata = metadata;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Metadata other && equals(other);
   }
}
