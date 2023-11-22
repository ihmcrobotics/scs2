package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput;
import us.ihmc.scs2.symbolic.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.EquationInput.SimpleDoubleVariable;

import java.util.List;

public final class EquationOperation<V extends EquationInput>
{
   private final String name;
   private final String description;
   private final List<EquationInput> inputs;
   private final V value;
   private final DoubleVariable derivative = new SimpleDoubleVariable();
   private final ValueOperation<V> valueOperation;
   private final DerivativeOperation<V> derivativeOperation;

   public EquationOperation(String name,
                            String description,
                            V result,
                            List<EquationInput> inputs,
                            ValueOperation<V> valueOperation,
                            DerivativeOperation<V> derivativeOperation)
   {
      this.name = name;
      this.description = description;
      this.value = result;
      this.inputs = inputs;
      this.valueOperation = valueOperation;
      this.derivativeOperation = derivativeOperation;
   }

   public void setTime(double time)
   {
      value.setTime(time);
      derivative.setTime(time);
      for (EquationInput input : inputs)
         input.setTime(time);
   }

   public void computeValue(double time)
   {
      valueOperation.computeValue(time, value);
   }

   public void computeDerivative(double time)
   {
      if (derivativeOperation == null)
         derivative.setValue(time, 0.0);
      else
         derivativeOperation.computeDerivative(time, value, derivative);
   }

   public void resetInputs()
   {
      value.reset();
      derivative.reset();
      for (EquationInput input : inputs)
         input.reset();
   }

   public void updatePreviousInputValues()
   {
      value.updatePreviousValue();
      derivative.updatePreviousValue();
      for (EquationInput input : inputs)
         input.updatePreviousValue();
   }

   public V getValue()
   {
      return value;
   }

   public DoubleVariable getDerivative()
   {
      return derivative;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return description;
   }

   public interface ValueOperation<V extends EquationInput>
   {
      /**
       * Computes the value of the operation.
       *
       * @param time         the current time.
       * @param resultToPack the variable to store the value into.
       */
      void computeValue(double time, V resultToPack);
   }

   public interface DerivativeOperation<V extends EquationInput>
   {
      /**
       * Computes the derivative of the value with respect to time.
       *
       * @param time         the current time.
       * @param value        the value to compute the derivative of. It is assumed that the value has been updated to the current time.
       * @param resultToPack the variable to store the derivative into.
       */
      void computeDerivative(double time, V value, DoubleVariable resultToPack);
   }
}