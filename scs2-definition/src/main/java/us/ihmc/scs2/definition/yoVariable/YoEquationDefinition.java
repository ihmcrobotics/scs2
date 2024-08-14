package us.ihmc.scs2.definition.yoVariable;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
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
       * The value, can either be a double value, an integer value, or a {@code YoVariable}.
       */
      private EquationInputDefinition value;

      public EquationAliasDefinition()
      {
      }

      public EquationAliasDefinition(String name, EquationInputDefinition value)
      {
         this.name = name;
         this.value = value;
      }

      public void setName(String name)
      {
         this.name = name;
      }

      public void setValue(EquationInputDefinition value)
      {
         this.value = value;
      }

      public String getName()
      {
         return name;
      }

      public EquationInputDefinition getValue()
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

   public static class EquationInputDefinition
   {
      private String value;
      private YoVariableDefinition yoVariableValue;
      private boolean isConstant;

      public EquationInputDefinition()
      {
      }

      public EquationInputDefinition(String value)
      {
         this(value, false);
      }

      public EquationInputDefinition(String value, boolean isConstant)
      {
         this.value = value;
         this.isConstant = isConstant;
      }

      public EquationInputDefinition(YoVariableDefinition yoVariableValue)
      {
         this.yoVariableValue = yoVariableValue;
      }

      public void setValue(String value)
      {
         this.value = value;
      }

      public void setYoVariableValue(YoVariableDefinition yoVariableValue)
      {
         this.yoVariableValue = yoVariableValue;
      }

      public void setConstant(boolean constant)
      {
         isConstant = constant;
      }

      public String getValue()
      {
         return value;
      }

      public YoVariableDefinition getYoVariableValue()
      {
         return yoVariableValue;
      }

      public String computeSimpleStringValue()
      {
         if (value != null)
            return value;
         else if (yoVariableValue != null)
            return yoVariableValue.getFullname();
         else
            return null;
      }

      public boolean isConstant()
      {
         return isConstant;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof EquationInputDefinition other)
         {
            if (!Objects.equals(value, other.value))
               return false;
            if (!Objects.equals(yoVariableValue, other.yoVariableValue))
               return false;
            if (isConstant != other.isConstant)
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
         return "value: " + value + ", yoVariableValue: " + yoVariableValue;
      }
   }
}
