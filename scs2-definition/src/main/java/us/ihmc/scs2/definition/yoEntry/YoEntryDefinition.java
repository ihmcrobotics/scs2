package us.ihmc.scs2.definition.yoEntry;

import javax.xml.bind.annotation.XmlElement;

public class YoEntryDefinition
{
   private String compositeType;
   private String compositeFullname;

   @XmlElement
   public void setCompositeType(String compositeType)
   {
      this.compositeType = compositeType;
   }

   @XmlElement
   public void setCompositeFullname(String compositeFullname)
   {
      this.compositeFullname = compositeFullname;
   }

   public String getCompositeType()
   {
      return compositeType;
   }

   public String getCompositeFullname()
   {
      return compositeFullname;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoEntryDefinition)
      {
         YoEntryDefinition other = (YoEntryDefinition) object;

         if (compositeType == null ? other.compositeType != null : !compositeType.equals(other.compositeType))
            return false;
         if (compositeFullname == null ? other.compositeFullname != null : !compositeFullname.equals(other.compositeFullname))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "type: " + compositeType + ", fullname: " + compositeFullname;
   }
}
