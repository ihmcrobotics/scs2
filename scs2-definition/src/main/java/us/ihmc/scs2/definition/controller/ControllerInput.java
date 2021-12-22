package us.ihmc.scs2.definition.controller;

import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;

public class ControllerInput
{
   private double time;
   private final MultiBodySystemReadOnly input;

   public ControllerInput(MultiBodySystemReadOnly input)
   {
      this.input = input;
   }

   public void setTime(double time)
   {
      this.time = time;
   }

   public double getTime()
   {
      return time;
   }

   public MultiBodySystemReadOnly getInput()
   {
      return input;
   }

   public MultiBodySystemBasics createCopy(ReferenceFrame rootFrame)
   {
      return MultiBodySystemBasics.clone(input, rootFrame);
   }

   public void readState(MultiBodySystemBasics multiBodySystemToUpdate)
   {
      List<? extends JointReadOnly> inputJoints = input.getAllJoints();
      List<? extends JointBasics> outputJoints = multiBodySystemToUpdate.getAllJoints();

      for (int jointIndex = 0; jointIndex < inputJoints.size(); jointIndex++)
      {
         JointReadOnly inputJoint = inputJoints.get(jointIndex);
         JointBasics outputJoint = outputJoints.get(jointIndex);

         outputJoint.setJointConfiguration(inputJoint);
         outputJoint.setJointTwist(inputJoint);
      }
   }
}
