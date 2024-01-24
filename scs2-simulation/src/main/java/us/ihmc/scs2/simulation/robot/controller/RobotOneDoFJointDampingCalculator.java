package us.ihmc.scs2.simulation.robot.controller;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrix;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RobotOneDoFJointDampingCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final List<JointCalculator> jointCalculators = new ArrayList<>();
   private final JointMatrixIndexProvider jointMatrixIndexProvider;

   public RobotOneDoFJointDampingCalculator(RobotInterface robot)
   {
      jointMatrixIndexProvider = robot.getJointMatrixIndexProvider();

      for (SimJointBasics joint : robot.getJointsToConsider())
      {
         if (joint instanceof SimOneDoFJointBasics)
         {
            jointCalculators.add(new JointCalculator((SimOneDoFJointBasics) joint, registry));
         }
      }
   }

   public void compute()
   {
      for (int i = 0; i < jointCalculators.size(); i++)
      {
         jointCalculators.get(i).doControl();
      }
   }

   public void compute(DMatrix tauToAppendTo)
   {
      compute();

      for (JointCalculator calculator : jointCalculators)
      {
         int jointIndex = jointMatrixIndexProvider.getJointDoFIndices(calculator.joint)[0];
         double currentValue = tauToAppendTo.get(jointIndex, 0);
         // Appending this tau to the current value
         tauToAppendTo.set(jointIndex, 0, currentValue + calculator.dampingEffort.getDoubleValue());
      }
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   private static class JointCalculator
   {
      private final SimOneDoFJointBasics joint;
      private final YoDouble dampingEffort;

      public JointCalculator(SimOneDoFJointBasics joint, YoRegistry registry)
      {
         this.joint = joint;
         dampingEffort = new YoDouble("tau_damping_" + joint.getName(), registry);
      }

      public void doControl()
      {
         double tauDamping = -joint.getDamping() * joint.getQd();
         dampingEffort.set(tauDamping);
         joint.setTau(joint.getTau() + tauDamping);
      }
   }
}
