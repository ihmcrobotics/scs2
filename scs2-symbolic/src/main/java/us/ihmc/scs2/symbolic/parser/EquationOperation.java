package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput;

import java.util.function.Consumer;

public final class EquationOperation<V extends EquationInput>
{
   private final String name;
   private final String description;
   private final V result;
   private final Consumer<V> calculation;

   protected EquationOperation(String name, String description, V result, Consumer<V> calculation)
   {
      this.name = name;
      this.description = description;
      this.result = result;
      this.calculation = calculation;
   }

   public void calculate()
   {
      calculation.accept(result);
   }

   public V getResult()
   {
      return result;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return description;
   }
}