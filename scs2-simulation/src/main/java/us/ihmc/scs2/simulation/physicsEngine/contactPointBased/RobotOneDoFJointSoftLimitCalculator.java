package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrix;
import us.ihmc.mecano.multiBodySystem.interfaces.JointMatrixIndexProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RobotOneDoFJointSoftLimitCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final List<JointCalculator> jointCalculators = new ArrayList<>();
   private final JointMatrixIndexProvider jointMatrixIndexProvider;


   public RobotOneDoFJointSoftLimitCalculator(RobotInterface robot)
   {
      jointMatrixIndexProvider = robot.getJointMatrixIndexProvider();

      for (SimJointBasics joint : robot.getJointsToConsider())
      {
         if (joint instanceof SimOneDoFJointBasics)
         {
            SimOneDoFJointBasics oneDoFJoint = (SimOneDoFJointBasics) joint;
            OneDoFJointDefinition jointDefinition = (OneDoFJointDefinition) robot.getRobotDefinition().getJointDefinition(oneDoFJoint.getName());

            if (hasPositionLimit(oneDoFJoint) && hasSoftLimitStopGains(jointDefinition))
               jointCalculators.add(new JointCalculator(oneDoFJoint, jointDefinition, registry));
         }
      }
   }

   public void compute()
   {
      for (int i = 0; i < jointCalculators.size(); i++)
      {
         jointCalculators.get(i).compute();
      }
   }

   /**
    * TODO: note about appending, implies .zero() must be called somewhere
    * @param tauToAppendTo
    */
   public void compute(DMatrix tauToAppendTo)
   {
      compute();

      for (JointCalculator calculator : jointCalculators)
      {
         int jointIndex = jointMatrixIndexProvider.getJointDoFIndices(calculator.joint)[0];
         double currentValue = tauToAppendTo.get(jointIndex, 0);
         // Appending this tau to the current value
         tauToAppendTo.set(jointIndex, 0, currentValue + calculator.jointLimitEffort.getDoubleValue());
      }
   }

   private static boolean hasPositionLimit(OneDoFJointReadOnly joint)
   {
      double lower = joint.getJointLimitLower();
      double upper = joint.getJointLimitUpper();

      if (!Double.isFinite(lower) || !Double.isFinite(upper))
         return false;
      else
         return lower < upper;
   }

   private static boolean hasSoftLimitStopGains(OneDoFJointDefinition jointDefinition)
   {
      double kp = jointDefinition.getKpSoftLimitStop();
      double kd = jointDefinition.getKdSoftLimitStop();

      if (Double.isFinite(kp) && kp > 0.0)
         return true;
      if (Double.isFinite(kd) && kd > 0.0)
         return true;

      return false;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   private static class JointCalculator
   {
      private final SimOneDoFJointBasics joint;

      private final YoDouble kp, kd;
      private final YoDouble jointLimitEffort;

      public JointCalculator(SimOneDoFJointBasics joint, OneDoFJointDefinition jointDefinition, YoRegistry registry)
      {
         this.joint = joint;

         kp = new YoDouble("kp_soft_limit_stop_" + joint.getName(), registry);
         kd = new YoDouble("kd_soft_limit_stop_" + joint.getName(), registry);
         if (Double.isFinite(jointDefinition.getKpSoftLimitStop()) && jointDefinition.getKpSoftLimitStop() > 0.0)
            kp.set(jointDefinition.getKpSoftLimitStop());
         if (Double.isFinite(jointDefinition.getKdSoftLimitStop()) && jointDefinition.getKdSoftLimitStop() > 0.0)
            kd.set(jointDefinition.getKdSoftLimitStop());
         jointLimitEffort = new YoDouble("tau_soft_limit_stop_" + joint.getName(), registry);
      }

      public void compute()
      {
         double q = joint.getQ();
         double qd = joint.getQd();
         double min = joint.getJointLimitLower();
         double max = joint.getJointLimitUpper();
         double kp = this.kp.getValue();
         double kd = this.kd.getValue();

         double tau = 0.0;

         if (q < min)
            tau = Math.max(0.0, kp * (min - q) - kd * qd);
         else if (q > max)
            tau = Math.min(0.0, kp * (max - q) - kd * qd);

         jointLimitEffort.set(tau);
         joint.setTau(joint.getTau() + tau);

      }
   }
}
