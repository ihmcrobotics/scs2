package us.ihmc.scs2.definition.yoVariable;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import java.util.ArrayList;
import java.util.List;

public class YoEquationDefinition
{
   private String name;
   private String description;
   private String equation;

   private List<EquationAliasDefinition> aliases = new ArrayList<>();

   public YoEquationDefinition()
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
   public void setEquation(String equation)
   {
      this.equation = equation;
   }

   @XmlElement
   public void setAliases(List<EquationAliasDefinition> aliases)
   {
      this.aliases = aliases;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return description;
   }

   public String getEquation()
   {
      return equation;
   }

   public List<EquationAliasDefinition> getAliases()
   {
      return aliases;
   }

   public static class EquationAliasDefinition
   {
      /**
       * The variable alias in the equation
       */
      private String name;
      /**
       * The name of the {@code YoVariable} to link to the equation.
       * <p>
       * It can be either the fullname (including namespace, e.g. {@code "root.Controller.myVariable"}) or
       * the simple name (without namespace, e.g. {@code "myVariable"}). In case of name duplicates
       * between {@code YoVariable}s, prefer using the fullname to guarantee which variable is linked.
       * </p>
       */
      private String variableName;

      public void setName(String name)
      {
         this.name = name;
      }

      public void setVariableName(String variableName)
      {
         this.variableName = variableName;
      }

      public String getName()
      {
         return name;
      }

      public String getVariableName()
      {
         return variableName;
      }
   }
}
