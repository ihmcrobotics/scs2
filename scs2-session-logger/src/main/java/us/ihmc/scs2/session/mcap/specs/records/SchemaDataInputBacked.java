package us.ihmc.scs2.session.mcap.specs.records;

import us.ihmc.scs2.session.mcap.input.MCAPDataInput;
import us.ihmc.scs2.session.mcap.specs.MCAP;

import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;

public class SchemaDataInputBacked implements Schema
{
   private final MCAPDataInput dataInput;
   private final int id;
   private final String name;
   private final String encoding;
   private final long dataLength;
   private final long dataOffset;
   private WeakReference<ByteBuffer> dataRef;

   public SchemaDataInputBacked(MCAPDataInput dataInput, long elementPosition, long elementLength)
   {
      this.dataInput = dataInput;

      dataInput.position(elementPosition);
      id = dataInput.getUnsignedShort();
      name = dataInput.getString();
      encoding = dataInput.getString();
      dataLength = dataInput.getUnsignedInt();
      dataOffset = dataInput.position();
      MCAP.checkLength(elementLength, getElementLength());
   }

   @Override
   public int id()
   {
      return id;
   }

   @Override
   public String name()
   {
      return name;
   }

   @Override
   public String encoding()
   {
      return encoding;
   }

   @Override
   public long dataLength()
   {
      return dataLength;
   }

   @Override
   public ByteBuffer dataBuffer()
   {
      ByteBuffer data = this.dataRef == null ? null : this.dataRef.get();

      if (data == null)
      {
         data = dataInput.getByteBuffer(dataOffset, (int) dataLength, false);
         dataRef = new WeakReference<>(data);
      }
      return data;
   }

   @Override
   public String toString()
   {
      return toString(0);
   }

   @Override
   public boolean equals(Object object)
   {
      return object instanceof Schema schema && Schema.super.equals(schema);
   }
}
