package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;

public class MetadataIndexDataInputBacked implements MetadataIndex
{
   private final MCAPDataInput dataInput;
   private final long metadataOffset;
   private final long metadataLength;
   private final String name;
   private WeakReference<Record> metadataRef;

   MetadataIndexDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      metadataOffset = MCAP.checkPositiveLong(dataInput.getLong(), "metadataOffset");
      metadataLength = MCAP.checkPositiveLong(dataInput.getLong(), "metadataLength");
      name = dataInput.getString();
      MCAP.checkLength(elementLength, getElementLength());
   }

   @Override
   public Record metadata()
   {
      Record metadata = metadataRef == null ? null : metadataRef.get();

      if (metadata == null)
      {
         metadata = new RecordDataInputBacked(dataInput, metadataOffset);
         metadataRef = new WeakReference<>(metadata);
      }
      return metadata;
   }

   @Override
   public long metadataOffset()
   {
      return metadataOffset;
   }

   @Override
   public long metadataLength()
   {
      return metadataLength;
   }

   @Override
   public String name()
   {
      return name;
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
