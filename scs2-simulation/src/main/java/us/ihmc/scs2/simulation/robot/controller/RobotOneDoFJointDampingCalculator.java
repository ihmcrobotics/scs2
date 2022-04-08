package us.ihmc.scs2.simulation.robot.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RobotOneDoFJointDampingCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final List<JointCalculator> jointCalculators = new ArrayList<>();

   public RobotOneDoFJointDampingCalculator(RobotInterface robot)
   {
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
