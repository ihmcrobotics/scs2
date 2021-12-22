package us.ihmc.scs2.definition.controller;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.interfaces.ControllerOutputBasics;
import us.ihmc.scs2.definition.state.JointState;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;

public class ControllerOutput implements ControllerOutputBasics
{
   private final MultiBodySystemReadOnly input;
   private final JointStateBasics[] jointOutputs;
   private final Map<String, JointStateBasics> jointOutputMap = new HashMap<>();

   public ControllerOutput(MultiBodySystemReadOnly input)
   {
      this.input = input;
      List<? extends JointReadOnly> allJoints = input.getAllJoints();
      jointOutputs = new JointStateBasics[allJoints.size()];
      for (int i = 0; i < allJoints.size(); i++)
      {
         JointReadOnly joint = allJoints.get(i);
         JointStateBasics output;

         if (joint instanceof OneDoFJointReadOnly)
            output = new OneDoFJointState();
         else
            output = new JointState(joint.getConfigurationMatrixSize(), joint.getDegreesOfFreedom());
         jointOutputs[i] = output;
         jointOutputMap.put(joint.getName(), output);
      }
   }

   @Override
   public MultiBodySystemReadOnly getInput()
   {
      return input;
   }

   @Override
   public JointStateBasics getJointOutput(String jointName)
   {
      return jointOutputMap.get(jointName);
   }

   @Override
   public JointStateBasics[] getJointOuputs()
   {
      return jointOutputs;
   }
}
