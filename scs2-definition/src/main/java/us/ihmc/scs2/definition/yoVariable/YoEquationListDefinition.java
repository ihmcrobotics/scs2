package us.ihmc.scs2.definition.yoVariable;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.ArrayList;
import java.util.List;

@XmlRootElement(name = "YoEquations")
public class YoEquationListDefinition
{
   private List<YoEquationDefinition> yoEquations = new ArrayList<>();

   public YoEquationListDefinition()
   {
   }

   public YoEquationListDefinition(List<YoEquationDefinition> yoEquations)
   {
      this.yoEquations = yoEquations;
   }

   @XmlElement
   public void setYoEquations(List<YoEquationDefinition> yoEquations)
   {
      this.yoEquations = yoEquations;
   }

   public List<YoEquationDefinition> getYoEquations()
   {
      return yoEquations;
   }

   @Override
   public String toString()
   {
      return yoEquations.toString();
   }
}
