package us.ihmc.scs2.definition.yoChart;

import java.util.ArrayList;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoChartGroupModel")
public class YoChartGroupModelDefinition
{
   private String name;
   private final List<YoChartIdentifierDefinition> chartIdentifiers = new ArrayList<>();

   public YoChartGroupModelDefinition()
   {
   }

   public YoChartGroupModelDefinition(String name)
   {
      setName(name);
   }

   public YoChartGroupModelDefinition(YoChartGroupModelDefinition other)
   {
      if (other == null)
         return;
      setName(other.name);
      setChartIdentifiers(other.chartIdentifiers);
   }

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement
   public void setChartIdentifiers(List<YoChartIdentifierDefinition> chartIdentifiers)
   {
      this.chartIdentifiers.clear();
      for (YoChartIdentifierDefinition chartId : chartIdentifiers)
         this.chartIdentifiers.add(chartId.clone());
   }

   public String getName()
   {
      return name;
   }

   public List<YoChartIdentifierDefinition> getChartIdentifiers()
   {
      return chartIdentifiers;
   }

   @Override
   public YoChartGroupModelDefinition clone()
   {
      return new YoChartGroupModelDefinition(this);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoChartGroupModelDefinition)
      {
         YoChartGroupModelDefinition other = (YoChartGroupModelDefinition) object;
         if (name == null ? other.name != null : !name.equals(other.name))
            return false;
         if (!chartIdentifiers.equals(other.chartIdentifiers))
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
      return "name " + name + ", " + chartIdentifiers.toString();
   }
}
