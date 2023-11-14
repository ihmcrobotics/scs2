package us.ihmc.scs2.definition.yoVariable;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class YoEquationDefinition
{
   private String name;
   private String description;
   private String equation;

   private List<EquationAliasDefinition> aliases = new ArrayList<>();

   public YoEquationDefinition()
   {
   }

   public void set(YoEquationDefinition other)
   {
      name = other.name;
      description = other.description;
      equation = other.equation;
      aliases.clear();
      aliases.addAll(other.aliases);
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

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoEquationDefinition other)
      {
         if (!Objects.equals(name, other.name))
            return false;
         if (!Objects.equals(description, other.description))
            return false;
         if (!Objects.equals(equation, other.equation))
            return false;
         if (!Objects.equals(aliases, other.aliases))
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
      return "name: " + name + ", description: " + description + ", equation: " + equation + ", aliases: " + aliases;
   }

   public static class EquationAliasDefinition
   {
      /**
       * The variable alias in the equation
       */
      private String name;
      /**
       * The value (double or integer) or the name of the {@code YoVariable} to link to the equation.
       * <p>
       * It can be either the fullname (including namespace, e.g. {@code "root.Controller.myVariable"}) or
       * the simple name (without namespace, e.g. {@code "myVariable"}). In case of name duplicates
       * between {@code YoVariable}s, prefer using the fullname to guarantee which variable is linked.
       * </p>
       */
      private String value;

      public EquationAliasDefinition()
      {
      }

      public EquationAliasDefinition(String name, String value)
      {
         this.name = name;
         this.value = value;
      }

      public void setName(String name)
      {
         this.name = name;
      }

      public void setValue(String value)
      {
         this.value = value;
      }

      public String getName()
      {
         return name;
      }

      public String getValue()
      {
         return value;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof EquationAliasDefinition other)
         {
            if (!Objects.equals(name, other.name))
               return false;
            if (!Objects.equals(value, other.value))
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
         return "name: " + name + ", value: " + value;
      }
   }
}
