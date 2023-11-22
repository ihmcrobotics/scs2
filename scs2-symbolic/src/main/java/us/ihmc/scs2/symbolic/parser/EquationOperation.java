package us.ihmc.scs2.symbolic.parser;

import us.ihmc.scs2.symbolic.EquationInput;

import java.util.List;

public abstract class EquationOperation<I extends EquationInput> implements EquationInput
{
   protected double time = Double.NaN;
   protected double previousTime = Double.NaN;

   private final String name;
   private final String description;
   protected final List<? extends I> inputs;

   public EquationOperation(String name, String description, List<? extends I> inputs)
   {
      this.name = name;
      this.description = description;
      this.inputs = inputs;
   }

   public void setTime(double time)
   {
      this.time = time;
      for (I input : inputs)
         input.setTime(time);
   }

   public abstract void computeValue(double time);

   public abstract void computeDerivative(double time);

   public void resetInputs()
   {
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
}