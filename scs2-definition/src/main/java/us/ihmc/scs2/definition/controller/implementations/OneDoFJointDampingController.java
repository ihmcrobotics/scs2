package us.ihmc.scs2.definition.controller.implementations;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class OneDoFJointDampingController implements Controller
{
   private final String controllerName;
   private final YoRegistry registry;
   private final DoubleProvider damping;
   private final OneDoFJointReadOnly[] joints;
   private final OneDoFJointStateBasics[] jointOutputs;

   private final YoDouble[] dampingEffort;

   public OneDoFJointDampingController(String controllerName, DoubleProvider damping, OneDoFJointReadOnly[] jointsToControl,
                                       OneDoFJointStateBasics[] jointOutputs, YoRegistry registry)
   {
      this.controllerName = controllerName;
      this.damping = damping;
      this.joints = jointsToControl;
      this.jointOutputs = jointOutputs;
      this.registry = registry;

      dampingEffort = new YoDouble[jointsToControl.length];

      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         dampingEffort[jointIndex] = new YoDouble("tau_damp_" + joints[jointIndex].getName(), registry);
      }
   }

   @Override
   public void doControl()
   {
      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         OneDoFJointReadOnly joint = joints[jointIndex];
         double tau = -damping.getValue() * joint.getQd();
         dampingEffort[jointIndex].set(tau);
         jointOutputs[jointIndex].addEffort(tau);
      }
   }

   @Override
   public String getName()
   {
      return controllerName;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
