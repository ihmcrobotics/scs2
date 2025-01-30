package us.ihmc.scs2.definition.yoComposite;

import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoCompositePatternList")
public class YoCompositePatternListDefinition
{
   private String name;
   private List<YoCompositePatternDefinition> yoCompositePatterns;

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   @XmlElement
   public void setYoCompositePatterns(List<YoCompositePatternDefinition> yoCompositePatterns)
   {
      this.yoCompositePatterns = yoCompositePatterns;
   }

   public String getName()
   {
      return name;
   }

   public List<YoCompositePatternDefinition> getYoCompositePatterns()
   {
      return yoCompositePatterns;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoCompositePatternListDefinition)
      {
         YoCompositePatternListDefinition other = (YoCompositePatternListDefinition) object;
         if (name == null ? other.name != null : !name.equals(other.name))
            return false;
         if (yoCompositePatterns == null ? other.yoCompositePatterns != null : !yoCompositePatterns.equals(other.yoCompositePatterns))
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
      return "name " + name + ", " + (yoCompositePatterns != null ? yoCompositePatterns.toString() : "empty");
   }
}
