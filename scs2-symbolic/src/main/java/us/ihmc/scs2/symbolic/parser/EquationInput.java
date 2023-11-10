package us.ihmc.scs2.symbolic.parser;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public interface EquationInput
{
   enum Type
   {
      INTEGER, DOUBLE;
   }

   Type getType();

   interface ScalarConstant extends EquationInput
   {
      DoubleSupplier toDoubleSupplier();
   }

   interface ScalarVariable extends ScalarConstant
   {
   }

   class DoubleConstant implements ScalarConstant, DoubleSupplier
   {
      private final double value;

      public DoubleConstant(double value)
      {
         this.value = value;
      }

      @Override
      public Type getType()
      {
         return Type.DOUBLE;
      }

      @Override
      public double getAsDouble()
      {
         return value;
      }

      @Override
      public DoubleSupplier toDoubleSupplier()
      {
         return this;
      }
   }

   class DoubleVariable implements ScalarVariable, DoubleSupplier
   {
      private double value;

      public DoubleVariable(double value)
      {
         this.value = value;
      }

      @Override
      public Type getType()
      {
         return Type.DOUBLE;
      }

      public void setValue(double value)
      {
         this.value = value;
      }

      @Override
      public double getAsDouble()
      {
         return value;
      }

      @Override
      public DoubleSupplier toDoubleSupplier()
      {
         return this;
      }
   }

   class IntegerConstant implements ScalarConstant, IntSupplier
   {
      private final int value;

      public IntegerConstant(int value)
      {
         this.value = value;
      }

      @Override
      public Type getType()
      {
         return Type.INTEGER;
      }

      @Override
      public int getAsInt()
      {
         return value;
      }

      @Override
      public DoubleSupplier toDoubleSupplier()
      {
         return () -> value;
      }
   }

   class IntegerVariable implements ScalarVariable, IntSupplier
   {
      private int value;

      public IntegerVariable(int value)
      {
         this.value = value;
      }

      @Override
      public Type getType()
      {
         return Type.INTEGER;
      }

      public void setValue(int value)
      {
         this.value = value;
      }

      @Override
      public int getAsInt()
      {
         return value;
      }

      @Override
      public DoubleSupplier toDoubleSupplier()
      {
         return () -> value;
      }
   }
}