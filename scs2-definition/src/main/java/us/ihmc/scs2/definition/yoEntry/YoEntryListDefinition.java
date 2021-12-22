package us.ihmc.scs2.definition.yoEntry;

import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoEntryList")
public class YoEntryListDefinition
{
   private String name;
   private List<YoEntryDefinition> yoEntries;

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement
   public void setYoEntries(List<YoEntryDefinition> yoEntries)
   {
      this.yoEntries = yoEntries;
   }

   public String getName()
   {
      return name;
   }

   public List<YoEntryDefinition> getYoEntries()
   {
      return yoEntries;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoEntryListDefinition)
      {
         YoEntryListDefinition other = (YoEntryListDefinition) object;

         if (name == null ? other.name != null : !name.equals(other.name))
            return false;
         if (yoEntries == null ? other.yoEntries != null : !yoEntries.equals(other.yoEntries))
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
      return "name: " + name + ", yoEntries: " + yoEntries;
   }
}
