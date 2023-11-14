package us.ihmc.scs2.symbolic;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Objects;
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

   static YoScalarVariableHolder<?> newYoVariable(YoVariable yoVariable)
   {
      Objects.requireNonNull(yoVariable, "YoVariable cannot be null.");
      if (yoVariable instanceof YoDouble)
         return new YoDoubleVariable((YoDouble) yoVariable);
      else if (yoVariable instanceof YoInteger)
         return new YoIntegerVariable((YoInteger) yoVariable);
      else
         throw new IllegalArgumentException("Unsupported YoVariable type: " + yoVariable.getClass().getSimpleName());
   }

   static YoScalarConstantHolder<?> newYoConstant(YoVariable yoVariable)
   {
      Objects.requireNonNull(yoVariable, "YoVariable cannot be null.");
      if (yoVariable instanceof YoDouble)
         return new YoDoubleConstant((YoDouble) yoVariable);
      else if (yoVariable instanceof YoInteger)
         return new YoIntegerConstant((YoInteger) yoVariable);
      else
         throw new IllegalArgumentException("Unsupported YoVariable type: " + yoVariable.getClass().getSimpleName());
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

   abstract class YoScalarConstantHolder<V extends YoVariable> implements ScalarConstant
   {
      protected final V yoVariable;

      public YoScalarConstantHolder(V yoVariable)
      {
         this.yoVariable = Objects.requireNonNull(yoVariable, "YoVariable cannot be null.");
      }
   }

   class YoDoubleConstant extends YoScalarConstantHolder<YoDouble> implements DoubleConstant
   {
      public YoDoubleConstant(YoDouble yoDouble)
      {
         super(yoDouble);
      }

      @Override
      public double getAsDouble()
      {
         return yoVariable.getValue();
      }

      @Override
      public String valueAsString()
      {
         return yoVariable.getFullNameString();
      }
   }

   class YoIntegerConstant extends YoScalarConstantHolder<YoInteger> implements IntegerConstant
   {
      public YoIntegerConstant(YoInteger yoInteger)
      {
         super(yoInteger);
      }

      @Override
      public int getAsInt()
      {
         return yoVariable.getValue();
      }

      @Override
      public String valueAsString()
      {
         return yoVariable.getFullNameString();
      }
   }

   abstract class YoScalarVariableHolder<V extends YoVariable> implements ScalarVariable
   {
      protected final V yoVariable;

      public YoScalarVariableHolder(V yoVariable)
      {
         this.yoVariable = Objects.requireNonNull(yoVariable, "YoVariable cannot be null.");
      }
   }

   class YoDoubleVariable extends YoScalarVariableHolder<YoDouble> implements DoubleVariable
   {
      public YoDoubleVariable(YoDouble yoDouble)
      {
         super(yoDouble);
      }

      @Override
      public void setValue(double value)
      {
         yoVariable.set(value);
      }

      @Override
      public double getAsDouble()
      {
         return yoVariable.getValue();
      }

      @Override
      public String valueAsString()
      {
         return yoVariable.getFullNameString();
      }
   }

   class YoIntegerVariable extends YoScalarVariableHolder<YoInteger> implements IntegerVariable
   {
      public YoIntegerVariable(YoInteger yoInteger)
      {
         super(yoInteger);
      }

      @Override
      public void setValue(int value)
      {
         yoVariable.set(value);
      }

      @Override
      public int getAsInt()
      {
         return yoVariable.getValue();
      }

      @Override
      public String valueAsString()
      {
         return yoVariable.getFullNameString();
      }
   }
}