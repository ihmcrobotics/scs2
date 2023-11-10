package us.ihmc.scs2.symbolic.parser.parser;

import us.ihmc.scs2.symbolic.parser.EquationInput;

import java.util.function.Consumer;

public final class EquationOperation<V extends EquationInput>
{
   private final String name;
   private final V result;
   private final Consumer<V> calculation;

   protected EquationOperation(String name, V result, Consumer<V> calculation)
   {
      this.name = name;
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
}