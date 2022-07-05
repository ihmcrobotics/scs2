package us.ihmc.scs2.definition.controller.interfaces;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.state.interfaces.JointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;

public interface ControllerOutputBasics extends ControllerOutputReadOnly
{
   default void clear()
   {
      for (JointStateBasics jointOutput : getJointOuputs())
         jointOutput.clear();
   }

   @Override
   default JointStateBasics getJointOutput(JointReadOnly joint)
   {
      return getJointOutput(joint.getName());
   }

   @Override
   JointStateBasics getJointOutput(String jointName);

   default OneDoFJointStateBasics getOneDoFJointOutput(OneDoFJointReadOnly joint)
   {
      return (OneDoFJointStateBasics) getJointOutput(joint);
   }

   default OneDoFJointStateBasics getOneDoFJointOutput(String jointName)
   {
      return (OneDoFJointStateBasics) getJointOutput(jointName);
   }

   default JointStateBasics[] getJointOutputs(JointReadOnly[] joints)
   {
      return Stream.of(joints).map(this::getJointOutput).toArray(JointStateBasics[]::new);
   }

   default List<JointStateBasics> getJointOutputs(List<? extends JointReadOnly> joints)
   {
      return joints.stream().map(this::getJointOutput).collect(Collectors.toList());
   }

   default OneDoFJointStateBasics[] getOneDoFJointOutputs(OneDoFJointReadOnly[] joints)
   {
      return Stream.of(joints).map(this::getOneDoFJointOutput).toArray(OneDoFJointStateBasics[]::new);
   }

   default List<OneDoFJointStateBasics> getOneDoFJointOutputs(List<? extends OneDoFJointReadOnly> joints)
   {
      return joints.stream().map(this::getOneDoFJointOutput).collect(Collectors.toList());
   }

   @Override
   JointStateBasics[] getJointOuputs();
}