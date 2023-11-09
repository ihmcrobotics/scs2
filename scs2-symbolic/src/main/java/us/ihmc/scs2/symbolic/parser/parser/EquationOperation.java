package us.ihmc.scs2.symbolic.parser.parser;

import us.ihmc.scs2.symbolic.parser.EquationVariable;
import us.ihmc.scs2.symbolic.parser.EquationVariable.EquationVariableDouble;
import us.ihmc.scs2.symbolic.parser.EquationVariable.EquationVariableInteger;
import us.ihmc.scs2.symbolic.parser.EquationVariable.EquationVariableScalar;

import java.util.function.Consumer;

public final class EquationOperation<V extends EquationVariable>
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

   public String name()
   {
      return name;
   }

   public static EquationOperation<?> multiply(EquationVariable A, EquationVariable B)
   {
      if (A instanceof EquationVariableInteger intA && B instanceof EquationVariableInteger intB)
         return new EquationOperation<>("multiply-ii", new EquationVariableInteger(0), result -> result.value = intA.value * intB.value);
      else if (A instanceof EquationVariableScalar scalarA && B instanceof EquationVariableScalar scalarB)
         return new EquationOperation<>("multiply-ss", new EquationVariableDouble(0), result -> result.value = scalarA.getAsDouble() * scalarB.getAsDouble());
      else
         throw new IllegalStateException("Unexpected types: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
   }

   public static EquationOperation<?> divide(EquationVariable A, EquationVariable B)
   {
      if (A instanceof EquationVariableInteger intA && B instanceof EquationVariableInteger intB)
         return new EquationOperation<>("divide-ii", new EquationVariableInteger(0), result -> result.value = intA.value / intB.value);
      else if (A instanceof EquationVariableScalar scalarA && B instanceof EquationVariableScalar scalarB)
         return new EquationOperation<>("divide-ss", new EquationVariableDouble(0), result -> result.value = scalarA.getAsDouble() / scalarB.getAsDouble());
      else
         throw new IllegalStateException("Unexpected types: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
   }

   public static EquationOperation<?> pow(EquationVariable A, EquationVariable B)
   {
      if (A instanceof EquationVariableScalar scalarA && B instanceof EquationVariableScalar scalarB)
         return new EquationOperation<>("pow-ss", new EquationVariableDouble(0), result -> result.value = Math.pow(scalarA.getAsDouble(), scalarB.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected types: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
   }

   public static EquationOperation<?> atan2(EquationVariable A, EquationVariable B)
   {
      if (A instanceof EquationVariableScalar scalarA && B instanceof EquationVariableScalar scalarB)
         return new EquationOperation<>("atan2-ss", new EquationVariableDouble(0), result -> result.value = Math.atan2(scalarA.getAsDouble(), scalarB.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected types: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
   }

   public static EquationOperation<?> sqrt(EquationVariable A)
   {
      if (A instanceof EquationVariableScalar scalarA)
         return new EquationOperation<>("sqrt-s", new EquationVariableDouble(0), result -> result.value = Math.sqrt(scalarA.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected type: A = " + A.getClass().getSimpleName());
   }

   public static EquationOperation<?> sin(EquationVariable A)
   {
      if (A instanceof EquationVariableScalar scalarA)
         return new EquationOperation<>("sin-s", new EquationVariableDouble(0), result -> result.value = Math.sin(scalarA.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected type: A = " + A.getClass().getSimpleName());
   }

   public static EquationOperation<?> cos(EquationVariable A)
   {
      if (A instanceof EquationVariableScalar scalarA)
         return new EquationOperation<>("cos-s", new EquationVariableDouble(0), result -> result.value = Math.cos(scalarA.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected type: A = " + A.getClass().getSimpleName());
   }

   public static EquationOperation<?> atan(EquationVariable A)
   {
      if (A instanceof EquationVariableScalar scalarA)
         return new EquationOperation<>("atan-s", new EquationVariableDouble(0), result -> result.value = Math.atan(scalarA.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected type: A = " + A.getClass().getSimpleName());
   }

   public static EquationOperation<?> exp(EquationVariable A)
   {
      if (A instanceof EquationVariableScalar scalarA)
         return new EquationOperation<>("exp-s", new EquationVariableDouble(0), result -> result.value = Math.exp(scalarA.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected type: A = " + A.getClass().getSimpleName());
   }

   public static EquationOperation<?> log(EquationVariable A)
   {
      if (A instanceof EquationVariableScalar scalarA)
         return new EquationOperation<>("log-s", new EquationVariableDouble(0), result -> result.value = Math.log(scalarA.getAsDouble()));
      else
         throw new IllegalStateException("Unexpected type: A = " + A.getClass().getSimpleName());
   }

   public static EquationOperation<?> add(EquationVariable A, EquationVariable B)
   {
      if (A instanceof EquationVariableInteger intA && B instanceof EquationVariableInteger intB)
         return new EquationOperation<>("add-ii", new EquationVariableInteger(0), result -> result.value = intA.value + intB.value);
      else if (A instanceof EquationVariableScalar scalarA && B instanceof EquationVariableScalar scalarB)
         return new EquationOperation<>("add-ss", new EquationVariableDouble(0), result -> result.value = scalarA.getAsDouble() + scalarB.getAsDouble());
      else
         throw new IllegalStateException("Unexpected types: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
   }

   public static EquationOperation<?> subtract(EquationVariable A, EquationVariable B)
   {
      if (A instanceof EquationVariableInteger intA && B instanceof EquationVariableInteger intB)
         return new EquationOperation<>("subtract-ii", new EquationVariableInteger(0), result -> result.value = intA.value - intB.value);
      else if (A instanceof EquationVariableScalar scalarA && B instanceof EquationVariableScalar scalarB)
         return new EquationOperation<>("subtract-ss", new EquationVariableDouble(0), result -> result.value = scalarA.getAsDouble() - scalarB.getAsDouble());
      else
         throw new IllegalStateException("Unexpected types: A = " + A.getClass().getSimpleName() + ", B = " + B.getClass().getSimpleName());
   }

   public static EquationOperation<?> copy(EquationVariable src, EquationVariable dst)
   {
      if (src instanceof EquationVariableInteger intSrc && dst instanceof EquationVariableInteger intDst)
         return new EquationOperation<>("copy-ii", intDst, result -> result.value = intSrc.value);
      if (src instanceof EquationVariableScalar scalarSrc && dst instanceof EquationVariableDouble doubleDst)
         return new EquationOperation<>("copy-sd", doubleDst, result -> result.value = scalarSrc.getAsDouble());
      else
         throw new RuntimeException("Unsupported copy types; src = " + src.getClass().getSimpleName() + " dst = " + dst.getClass().getSimpleName());
   }

   public static EquationOperation<?> abs(EquationVariable A)
   {
      if (A instanceof EquationVariableInteger intA)
         return new EquationOperation<>("abs-i", new EquationVariableInteger(0), result -> result.value = Math.abs(intA.value));
      else if (A instanceof EquationVariableScalar scalarA)
         return new EquationOperation<>("abs-s", new EquationVariableDouble(0), result -> result.value = Math.abs(scalarA.getAsDouble()));
      else
      {
         throw new IllegalStateException("Unexpected type: A = " + A.getClass().getSimpleName());
      }
   }
}