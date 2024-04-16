package us.ihmc.scs2.session.mcap.specs.records;

import java.nio.ByteBuffer;

public class MutableSchema implements Schema
{
   private int id;
   private String name;
   private String encoding;
   private byte[] data;

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
   public String name()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   @Override
   public String encoding()
   {
      return encoding;
   }

   public void setEncoding(String encoding)
   {
      this.encoding = encoding;
   }

   @Override
   public long dataLength()
   {
      return data.length;
   }

   @Override
   public byte[] dataArray()
   {
      return data;
   }

   public void setData(byte[] data)
   {
      this.data = data;
   }

   @Override
   public ByteBuffer dataBuffer()
   {
      return ByteBuffer.wrap(data);
   }
}
