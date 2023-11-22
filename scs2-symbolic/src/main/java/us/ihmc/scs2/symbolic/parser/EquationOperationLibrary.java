package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput.DoubleVariable;
import us.ihmc.scs2.symbolic.EquationInput.ScalarInput;

import java.util.List;

public class EquationOperationLibrary
{
   public static final class AddDoubleOperation extends EquationOperation<ScalarInput> implements DoubleVariable
   {
      private double value = Double.NaN;
      private double derivative = Double.NaN;

      public AddDoubleOperation(ScalarInput A, ScalarInput B)
      {
         super("add", "Add two doubles.", List.of(A, B));
      }

      @Override
      public void computeValue(double time)
      {
         value = 0.0;
         for (int i = 0; i < getNumberOfInputs(); i++)
            value += getInput(i).getValueAsDouble();
      }

      @Override
      public void computeDerivative(double time)
      {
         derivative = 0.0;
         for (int i = 0; i < getNumberOfInputs(); i++)
            derivative += getInput(i).getValueDot();
      }
   }
}
