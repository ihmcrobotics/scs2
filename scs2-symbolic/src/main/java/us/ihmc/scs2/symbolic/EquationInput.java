package us.ihmc.scs2.symbolic;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationInputDefinition;

public interface EquationInput
{
   enum InputType
   {
      INTEGER, DOUBLE
   }

   /**
    * Main purpose is to reset the previous value of a variable.
    */
   default void reset()
   {
   }

   void setTime(double time);

   /**
    * Main purpose is to update the previous value of a variable.
    * <p>
    * It is assumed this method is called to save the current value of a variable before it is updated.
    * </p>
    */
   default void updatePreviousValue()
   {
   }

   InputType getType();

   String valueAsString();

   EquationInputDefinition toInputDefinition();

   static ScalarVariable newVariable(InputType type)
   {
      return switch (type)
      {
         case INTEGER -> new SimpleIntegerVariable();
         case DOUBLE -> new SimpleDoubleVariable();
      };
   }

   static DoubleVariable newDoubleVariable()
   {
      return new SimpleDoubleVariable();
   }

   static IntegerVariable newIntegerVariable()
   {
      return new SimpleIntegerVariable();
   }

   static ScalarInput parseConstant(InputType type, String valueAsString)
   {
      return switch (type)
      {
         case INTEGER -> new SimpleIntegerConstant(Integer.parseInt(valueAsString));
         case DOUBLE -> new SimpleDoubleConstant(Double.parseDouble(valueAsString));
      };
   }

   interface ScalarInput extends EquationInput
   {
      /**
       * @return the value of this input as a {@code double}.
       */
      double getValueAsDouble();

      /**
       * @return the value of this input as an {@code int}.
       */
      int getValueAsInteger();

      double getValueDot();
   }

   interface ScalarVariable extends ScalarInput
   {
   }

   interface DoubleInput extends ScalarInput
   {
      @Override
      default InputType getType()
      {
         return InputType.DOUBLE;
      }

      double getTime();

      double getValue();

      @Override
      default double getValueAsDouble()
      {
         return getValue();
      }

      @Override
      default int getValueAsInteger()
      {
         return (int) getValue();
      }

      double getPreviousTime();

      double getPreviousValue();

      @Override
      default double getValueDot()
      {
         if (Double.isNaN(getPreviousTime()) || getTime() == getPreviousTime())
            return 0.0;
         else
            return (getValue() - getPreviousValue()) / (getTime() - getPreviousTime());
      }

      @Override
      default String valueAsString()
      {
         return Double.toString(getValue());
      }
   }

   interface DoubleConstant extends DoubleInput
   {
      @Override
      default void setTime(double time)
      {
      }

      @Override
      default double getTime()
      {
         return Double.NaN;
      }

      @Override
      double getValue();

      @Override
      default double getPreviousTime()
      {
         return Double.NaN;
      }

      @Override
      default double getPreviousValue()
      {
         return getValue();
      }

      @Override
      default double getValueDot()
      {
         return 0.0;
      }

      @Override
      default String valueAsString()
      {
         return Double.toString(getValue());
      }
   }

   interface DoubleVariable extends ScalarVariable, DoubleInput
   {
      void setValue(double time, double value);
   }

   interface IntegerInput extends ScalarInput
   {
      @Override
      default InputType getType()
      {
         return InputType.INTEGER;
      }

      double getTime();

      int getValue();

      @Override
      default double getValueAsDouble()
      {
         return getValue();
      }

      @Override
      default int getValueAsInteger()
      {
         return getValue();
      }

      double getPreviousTime();

      int getPreviousValue();

      @Override
      default double getValueDot()
      {
         if (Double.isNaN(getPreviousTime()))
            return 0.0;
         else
            return (getValue() - getPreviousValue()) / (getTime() - getPreviousTime());
      }

      @Override
      default String valueAsString()
      {
         return Integer.toString(getValue());
      }
   }

   interface IntegerConstant extends IntegerInput
   {
      @Override
      default void setTime(double time)
      {
      }

      @Override
      default double getTime()
      {
         return Double.NaN;
      }

      @Override
      int getValue();

      @Override
      default double getPreviousTime()
      {
         return Double.NaN;
      }

      @Override
      default int getPreviousValue()
      {
         return getValue();
      }

      @Override
      default double getValueDot()
      {
         return 0.0;
      }

      @Override
      default String valueAsString()
      {
         return Integer.toString(getValue());
      }
   }

   interface IntegerVariable extends ScalarVariable, IntegerInput
   {
      void setValue(double time, int value);
   }

   class SimpleDoubleConstant implements DoubleConstant
   {
      private final double value;

      public SimpleDoubleConstant(double value)
      {
         this.value = value;
      }

      @Override
      public double getValue()
      {
         return value;
      }

      @Override
      public EquationInputDefinition toInputDefinition()
      {
         return new EquationInputDefinition(Double.toString(value), true);
      }

      @Override
      public String toString()
      {
         return Double.toString(value);
      }
   }

   class SimpleDoubleVariable implements DoubleVariable
   {
      private double value = Double.NaN;
      private double time = Double.NaN;
      private double previousValue = Double.NaN;
      private double previousTime = Double.NaN;

      public SimpleDoubleVariable()
      {
      }

      @Override
      public void reset()
      {
         time = Double.NaN;
         value = Double.NaN;
         previousTime = Double.NaN;
         previousValue = Double.NaN;
      }

      @Override
      public void updatePreviousValue()
      {
         previousTime = this.time;
         previousValue = value;
      }

      @Override
      public void setTime(double time)
      {
         this.time = time;
      }

      @Override
      public void setValue(double time, double value)
      {
         this.time = time;
         this.value = value;
      }

      @Override
      public double getTime()
      {
         return time;
      }

      @Override
      public double getValue()
      {
         return value;
      }

      @Override
      public double getPreviousTime()
      {
         return previousTime;
      }

      @Override
      public double getPreviousValue()
      {
         if (Double.isNaN(previousValue))
            return value;
         else
            return previousValue;
      }

      @Override
      public EquationInputDefinition toInputDefinition()
      {
         return new EquationInputDefinition(Double.toString(value), false);
      }

      @Override
      public String toString()
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
      public int getValue()
      {
         return value;
      }

      @Override
      public EquationInputDefinition toInputDefinition()
      {
         return new EquationInputDefinition(Integer.toString(value), true);
      }

      @Override
      public String toString()
      {
         return Integer.toString(value);
      }
   }

   class SimpleIntegerVariable implements IntegerVariable
   {
      private double time = Double.NaN;
      private int value = Integer.MIN_VALUE;
      private double previousTime = Double.NaN;
      private int previousValue = Integer.MIN_VALUE;

      public SimpleIntegerVariable()
      {
      }

      @Override
      public void reset()
      {
         time = Double.NaN;
         value = Integer.MIN_VALUE;
         previousTime = Double.NaN;
         previousValue = Integer.MIN_VALUE;
      }

      @Override
      public void updatePreviousValue()
      {
         previousTime = this.time;
         previousValue = value;
      }

      @Override
      public void setTime(double time)
      {
         this.time = time;
      }

      @Override
      public void setValue(double time, int value)
      {
         this.time = time;
         this.value = value;
      }

      @Override
      public double getTime()
      {
         return time;
      }

      @Override
      public int getValue()
      {
         return value;
      }

      @Override
      public double getPreviousTime()
      {
         return previousTime;
      }

      @Override
      public int getPreviousValue()
      {
         if (previousValue == Integer.MIN_VALUE)
            return value;
         else
            return previousValue;
      }

      @Override
      public EquationInputDefinition toInputDefinition()
      {
         return new EquationInputDefinition(Integer.toString(value), false);
      }

      @Override
      public String toString()
      {
         return Integer.toString(value);
      }
   }
}