package us.ihmc.scs2.symbolic.parser;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public abstract class EquationVariable
{
   public enum Type
   {
      INTEGER, DOUBLE;
   }

   public Type type;

   protected EquationVariable(Type type)
   {
      this.type = type;
   }

   public Type getType()
   {
      return type;
   }

   public abstract static class EquationVariableScalar extends EquationVariable implements DoubleSupplier
   {
      protected EquationVariableScalar(Type type)
      {
         super(type);
      }

      @Override
      public abstract double getAsDouble();
   }

   public static class EquationVariableDouble extends EquationVariableScalar
   {
      public double value;

      public EquationVariableDouble(double value)
      {
         super(Type.DOUBLE);

         this.value = value;
      }

      @Override
      public double getAsDouble()
      {
         return value;
      }
   }

   public static class EquationVariableInteger extends EquationVariableScalar implements IntSupplier
   {
      public int value;

      public EquationVariableInteger(int value)
      {
         super(Type.INTEGER);
         this.value = value;
      }

      @Override
      public double getAsDouble()
      {
         return value;
      }

      @Override
      public int getAsInt()
      {
         return value;
      }
   }
}