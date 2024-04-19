package us.ihmc.scs2.session.mcap.specs.records;

import java.util.List;

public class MutableMetadata implements Metadata
{
   private String name;
   private List<StringPair> metadata;

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
      long elementLength = Integer.BYTES + name.length();
      elementLength += Integer.BYTES;
      for (int i = 0; i < metadata.size(); i++)
         elementLength += metadata.get(i).getElementLength();
      return elementLength;
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
