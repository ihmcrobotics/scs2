package us.ihmc.scs2.definition.yoEntry;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

@XmlRootElement(name = "YoEntryConfiguration")
public class YoEntryConfigurationDefinition
{
   private List<YoEntryListDefinition> yoEntryLists;

   public YoEntryConfigurationDefinition()
   {
   }

   public YoEntryConfigurationDefinition(YoEntryListDefinition yoEntryList)
   {
      setYoEntryLists(Collections.singletonList(yoEntryList));
   }

   public YoEntryConfigurationDefinition(List<YoEntryListDefinition> yoEntryLists)
   {
      setYoEntryLists(yoEntryLists);
   }

   @XmlElement
   public void setYoEntryLists(List<YoEntryListDefinition> yoEntryLists)
   {
      this.yoEntryLists = yoEntryLists;
   }

   public List<YoEntryListDefinition> getYoEntryLists()
   {
      return yoEntryLists;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoEntryConfigurationDefinition)
      {
         YoEntryConfigurationDefinition other = (YoEntryConfigurationDefinition) object;

         if (yoEntryLists == null ? other.yoEntryLists != null : !yoEntryLists.equals(other.yoEntryLists))
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
      return EuclidCoreIOTools.getCollectionString("\n", yoEntryLists, Object::toString);
   }
}
