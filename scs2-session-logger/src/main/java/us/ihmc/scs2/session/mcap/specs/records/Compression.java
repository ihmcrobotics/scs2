package us.ihmc.scs2.session.mcap.specs.records;

public enum Compression
{
   NONE(""), LZ4("lz4"), ZSTD("zstd");

   private final String name;
   private final int length;

   Compression(String name)
   {
      this.name = name;
      length = name.getBytes().length + Integer.BYTES;
   }

   public int getLength()
   {
      return length;
   }

   public String getName()
   {
      return name;
   }

   public static Compression fromString(String name)
   {
      return switch (name.trim().toLowerCase())
      {
         case "" -> NONE;
         case "lz4" -> LZ4;
         case "zstd" -> ZSTD;
         default -> throw new IllegalArgumentException("Unsupported compression: " + name);
      };
   }
}
