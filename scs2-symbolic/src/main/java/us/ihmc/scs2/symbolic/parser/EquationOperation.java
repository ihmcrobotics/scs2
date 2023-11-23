package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition.EquationInputDefinition;
import us.ihmc.scs2.symbolic.EquationInput;

import java.util.List;

public abstract class EquationOperation<I extends EquationInput> implements EquationInput
{
   protected double time = Double.NaN;
   protected double previousTime = Double.NaN;
   protected boolean derivativeComputed = false;

   private final String name;
   private final String description;
   protected final List<? extends I> inputs;

   public EquationOperation(String name, String description, List<? extends I> inputs)
   {
      this.name = name;
      this.description = description;
      this.inputs = inputs;
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
      for (I input : inputs)
         input.setTime(time);
   }

   public void updateValue(double time)
   {
      setTime(time);
      computeValue(time);
      derivativeComputed = false;
   }

   protected abstract void computeValue(double time);

   protected abstract void computeDerivative(double time);

   @Override
   public void reset()
   {
      time = Double.NaN;
      previousTime = Double.NaN;
      for (I input : inputs)
         input.reset();
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return description;
   }

   public int getNumberOfInputs()
   {
      return inputs.size();
   }

   public I getInput(int index)
   {
      return inputs.get(index);
   }

   public static abstract class DoubleEquationOperation extends EquationOperation<ScalarInput> implements DoubleInput
   {
      protected double value = Double.NaN;
      protected double derivative = Double.NaN;
      protected double previousValue = Double.NaN;

      public DoubleEquationOperation(String name, String description, List<? extends ScalarInput> inputs)
      {
         super(name, description, inputs);
      }

      @Override
      public void reset()
      {
         time = Double.NaN;
         value = Double.NaN;
         previousTime = Double.NaN;
         previousValue = Double.NaN;

         for (int i = 0; i < getNumberOfInputs(); i++)
            getInput(i).reset();
      }

      @Override
      public void updatePreviousValue()
      {
         previousTime = this.time;
         previousValue = value;

         for (int i = 0; i < getNumberOfInputs(); i++)
            getInput(i).updatePreviousValue();
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
         return previousValue;
      }

      @Override
      public double getValueDot()
      {
         if (!derivativeComputed)
         {
            computeDerivative(time);
            derivativeComputed = true;
         }
         return derivative;
      }

      @Override
      public EquationInputDefinition toInputDefinition()
      {
         return null;
      }

      @Override
      public String valueAsString()
      {
         return DoubleInput.super.valueAsString();
      }
   }

   public static abstract class IntegerEquationOperation extends EquationOperation<ScalarInput> implements IntegerInput
   {
      protected int value = 0;
      protected double derivative = 0;
      protected int previousValue = 0;

      public IntegerEquationOperation(String name, String description, List<? extends ScalarInput> inputs)
      {
         super(name, description, inputs);
      }

      @Override
      public void reset()
      {
         time = Double.NaN;
         value = 0;
         previousTime = Double.NaN;
         previousValue = 0;

         for (int i = 0; i < getNumberOfInputs(); i++)
            getInput(i).reset();
      }

      @Override
      public void updatePreviousValue()
      {
         previousTime = this.time;
         previousValue = value;

         for (int i = 0; i < getNumberOfInputs(); i++)
            getInput(i).updatePreviousValue();
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
         return previousValue;
      }

      @Override
      public double getValueDot()
      {
         if (!derivativeComputed)
         {
            computeDerivative(time);
            derivativeComputed = true;
         }
         return derivative;
      }

      @Override
      public EquationInputDefinition toInputDefinition()
      {
         return null;
      }

      @Override
      public String valueAsString()
      {
         return IntegerInput.super.valueAsString();
      }
   }
}