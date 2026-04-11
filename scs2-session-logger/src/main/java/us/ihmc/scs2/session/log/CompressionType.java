package us.ihmc.scs2.session.log;

public enum CompressionType
{
   NONE, SNAPPY, ZSTD;

   public static CompressionType fromString(String value)
   {
      return switch (value.trim().toLowerCase())
      {
         case "", "none" -> NONE;
         case "snappy" -> SNAPPY;
         case "zstd" -> ZSTD;
         default -> throw new IllegalArgumentException("Unsupported compression type: " + value);
      };
   }
}
