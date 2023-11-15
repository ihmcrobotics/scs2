package us.ihmc.scs2.symbolic;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public interface EquationInput
{
   enum Type
   {
      INTEGER, DOUBLE;
   }

   Type getType();

   String valueAsString();

   static ScalarVariable newVariable(Type type)
   {
      return switch (type)
      {
         case INTEGER -> new SimpleIntegerVariable(0);
         case DOUBLE -> new SimpleDoubleVariable(0.0);
      };
   }

   interface ScalarConstant extends EquationInput
   {
      DoubleSupplier toDoubleSupplier();
   }

   interface ScalarVariable extends ScalarConstant
   {
   }

   interface DoubleConstant extends ScalarConstant, DoubleSupplier
   {
      @Override
      default Type getType()
      {
         return Type.DOUBLE;
      }

      @Override
      default DoubleSupplier toDoubleSupplier()
      {
         return this;
      }
   }

   interface DoubleVariable extends ScalarVariable, DoubleConstant
   {
      void setValue(double value);
   }

   interface IntegerConstant extends ScalarConstant, IntSupplier
   {
      @Override
      default Type getType()
      {
         return Type.INTEGER;
      }

      @Override
      default DoubleSupplier toDoubleSupplier()
      {
         return this::getAsInt;
      }
   }

   interface IntegerVariable extends ScalarVariable, IntegerConstant
   {
      void setValue(int value);
   }

   class SimpleDoubleConstant implements DoubleConstant
   {
      private final double value;

      public SimpleDoubleConstant(double value)
      {
         this.value = value;
      }

      @Override
      public double getAsDouble()
      {
         return value;
      }

      @Override
      public String valueAsString()
      {
         return Double.toString(value);
      }
   }

   class SimpleDoubleVariable implements DoubleVariable
   {
      private double value;

      public SimpleDoubleVariable(double value)
      {
         this.value = value;
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
      public String valueAsString()
      {
         return Double.toString(value);
      }
   }

   class SimpleIntegerConstant implements IntegerConstant
   {
      private final int value;

      public SimpleIntegerConstant(int value)
      {
         this.value = value;
      }

      @Override
      public int getAsInt()
      {
         return value;
      }

      @Override
      public String valueAsString()
      {
         return Integer.toString(value);
      }
   }

   class SimpleIntegerVariable implements IntegerVariable
   {
      private int value;

      public SimpleIntegerVariable(int value)
      {
         this.value = value;
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
      public String valueAsString()
      {
         return Integer.toString(value);
      }
   }
}