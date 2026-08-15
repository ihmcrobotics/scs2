package us.ihmc.scs2.simulation.physicsEngine.impulseBased;

import java.util.List;

public interface MultiContactImpulseCalculatorStepListener
{
   void hasStepped(CurrentStepInfo info);

   interface CurrentStepInfo
   {
      Step getStep();

      List<? extends ImpulseBasedConstraintCalculator> getAllCalculators();
   }

   enum Step
   {
      START, INITIALIZE, FILTER_CALCULATOR, COMPUTE_INERTIA, SOLVER_STEP, SOLVER_FINALIZE;
   }
}
