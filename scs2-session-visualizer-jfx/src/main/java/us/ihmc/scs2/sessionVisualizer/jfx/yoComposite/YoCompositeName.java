package us.ihmc.scs2.sessionVisualizer.jfx.yoComposite;

public class YoCompositeName
{
   private final String prefix;
   private final String suffix;
   private int hashCode = 0;

   public YoCompositeName(String prefix, String suffix)
   {
      this.prefix = prefix;
      this.suffix = suffix;
   }

   public String getPrefix()
   {
      return prefix;
   }

   public String getSuffix()
   {
      return suffix;
   }

   public String getName()
   {
      return prefix + suffix;
   }

   @Override
   public int hashCode()
   {
      if (hashCode == 0)
         hashCode = 31 * prefix.hashCode() + suffix.hashCode();
      return hashCode;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoCompositeName)
      {
         YoCompositeName other = (YoCompositeName) object;
         return prefix.equals(other.getPrefix()) && suffix.equals(other.getSuffix());
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "Prefix: " + prefix + ", suffix: " + suffix;
   }
}