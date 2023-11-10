package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.symbolic.parser.YoLibrary;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public interface EquationInput
{
   enum Type
   {
      INTEGER, DOUBLE;
   }

   Type getType();

   default boolean isUndefined()
   {
      return false;
   }

   static ScalarVariable newVariable(Type type)
   {
      return switch (type)
      {
         case INTEGER -> new SimpleIntegerVariable(0);
         case DOUBLE -> new SimpleDoubleVariable(0.0);
      };
   }

   static YoScalarVariableHolder newYoVariable(String variableName, YoLibrary library, Type type)
   {
      return switch (type)
      {
         case INTEGER -> new YoIntegerVariable(variableName, library);
         case DOUBLE -> new YoDoubleVariable(variableName, library);
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
         return () -> getAsInt();
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
   }

   abstract class YoScalarVariableHolder implements ScalarVariable
   {
      protected final String variableName;
      protected final YoLibrary library;

      public YoScalarVariableHolder(String variableName, YoLibrary library)
      {
         this.variableName = variableName;
         this.library = library;
      }

      public abstract boolean searchYoVariable();

      @Override
      public abstract boolean isUndefined();
   }

   class YoDoubleVariable extends YoScalarVariableHolder implements DoubleVariable
   {
      private YoDouble yoDouble;

      public YoDoubleVariable(String variableName, YoLibrary library)
      {
         super(variableName, library);
      }

      @Override
      public boolean searchYoVariable()
      {
         if (yoDouble == null)
            yoDouble = library.searchYoDouble(variableName);
         return yoDouble != null;
      }

      @Override
      public void setValue(double value)
      {
         searchYoVariable(); // TODO Need to throttle the search.
         if (yoDouble != null)
            yoDouble.set(value);
      }

      @Override
      public double getAsDouble()
      {
         searchYoVariable(); // TODO Need to throttle the search.
         return yoDouble == null ? Double.NaN : yoDouble.getValue();
      }

      @Override
      public boolean isUndefined()
      {
         return yoDouble == null;
      }
   }

   class YoIntegerVariable extends YoScalarVariableHolder implements IntegerVariable
   {
      private YoDouble yoInteger;

      public YoIntegerVariable(String variableName, YoLibrary library)
      {
         super(variableName, library);
      }

      @Override
      public boolean searchYoVariable()
      {
         if (yoInteger == null)
            yoInteger = library.searchYoDouble(variableName);
         return yoInteger != null;
      }

      @Override
      public void setValue(int value)
      {
         searchYoVariable(); // TODO Need to throttle the search.
         if (yoInteger != null)
            yoInteger.set(value);
      }

      @Override
      public int getAsInt()
      {
         searchYoVariable(); // TODO Need to throttle the search.
         return yoInteger == null ? 0 : (int) yoInteger.getValue();
      }

      @Override
      public boolean isUndefined()
      {
         return yoInteger == null;
      }
   }
}