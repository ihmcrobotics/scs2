package us.ihmc.scs2.definition.yoVariable;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public abstract class YoVariableDefinition
{
   private String name;
   private String description;
   private double lowerBound;
   private double upperBound;

   public YoVariableDefinition()
   {
   }

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlAttribute
   public void setDescription(String description)
   {
      this.description = description;
   }

   @XmlElement
   public void setLowerBound(double lowerBound)
   {
      this.lowerBound = lowerBound;
   }

   @XmlElement
   public void setUpperBound(double upperBound)
   {
      this.upperBound = upperBound;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return description;
   }

   public double getLowerBound()
   {
      return lowerBound;
   }

   public double getUpperBound()
   {
      return upperBound;
   }

   @Override
   public String toString()
   {
      return "name: " + name + ", description: " + description + ", lowerBound: " + lowerBound + ", upperBound: " + upperBound;
   }
}
