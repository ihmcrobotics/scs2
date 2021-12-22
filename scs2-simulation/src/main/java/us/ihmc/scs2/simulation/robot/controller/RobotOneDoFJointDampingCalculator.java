package us.ihmc.scs2.simulation.robot.controller;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
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
            SimOneDoFJointBasics oneDoFJoint = (SimOneDoFJointBasics) joint;
            OneDoFJointDefinition jointDefinition = (OneDoFJointDefinition) robot.getRobotDefinition().getJointDefinition(oneDoFJoint.getName());

            if (jointDefinition.getDamping() > 0.0)
               jointCalculators.add(new JointCalculator(oneDoFJoint, jointDefinition, registry));
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
      private final YoDouble damping;
      private final YoDouble dampingEffort;

      public JointCalculator(SimOneDoFJointBasics joint, OneDoFJointDefinition jointDefinition, YoRegistry registry)
      {
         this.joint = joint;

         damping = new YoDouble("damping_" + joint.getName(), registry);
         damping.set(jointDefinition.getDamping());
         dampingEffort = new YoDouble("tau_damping_" + joint.getName(), registry);
      }

      public void doControl()
      {
         double tauDamping = -damping.getValue() * joint.getQd();
         dampingEffort.set(tauDamping);
         joint.setTau(joint.getTau() + tauDamping);
      }
   }
}
