package us.ihmc.scs2.definition.controller.interfaces;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.scs2.definition.state.interfaces.JointStateReadOnly;

public interface ControllerOutputReadOnly
{
   MultiBodySystemReadOnly getInput();

   default JointStateReadOnly getJointOutput(JointReadOnly joint)
   {
      return getJointOutput(joint.getName());
   }
   
   JointStateReadOnly getJointOutput(String jointName);

   JointStateReadOnly[] getJointOuputs();
}
