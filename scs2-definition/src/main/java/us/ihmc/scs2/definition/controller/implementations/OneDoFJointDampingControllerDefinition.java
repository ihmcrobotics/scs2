package us.ihmc.scs2.definition.controller.implementations;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class OneDoFJointDampingControllerDefinition implements ControllerDefinition
{
   private String controllerName;
   private DoubleProvider damping;
   private List<String> namesOfJointsToControl;
   private YoRegistry controllerRegistry;

   public OneDoFJointDampingControllerDefinition setControllerName(String name)
   {
      controllerName = name;
      controllerRegistry = new YoRegistry(name);
      return this;
   }

   public OneDoFJointDampingControllerDefinition setDamping(DoubleProvider damping)
   {
      this.damping = damping;
      return this;
   }

   public OneDoFJointDampingControllerDefinition createDampingVariable(String variableName, double initialValue)
   {
      Objects.requireNonNull(controllerName);
      YoDouble yoDamping = new YoDouble(variableName, controllerRegistry);
      yoDamping.set(initialValue);
      this.damping = yoDamping;
      return this;
   }

   public OneDoFJointDampingControllerDefinition addJointToControl(String jointName)
   {
      if (namesOfJointsToControl == null)
         namesOfJointsToControl = new ArrayList<>();
      namesOfJointsToControl.add(jointName);
      return this;
   }

   public OneDoFJointDampingControllerDefinition addJointsToControl(String... jointNames)
   {
      for (String jointName : jointNames)
         addJointToControl(jointName);
      return this;
   }

   public OneDoFJointDampingControllerDefinition addJointsToControl(Iterable<String> jointNames)
   {
      for (String jointName : jointNames)
         addJointToControl(jointName);
      return this;
   }

   @Override
   public Controller newController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {
      Objects.requireNonNull(controllerName);
      Objects.requireNonNull(damping);
      Objects.requireNonNull(namesOfJointsToControl);
      Objects.requireNonNull(controllerRegistry);

      Map<String, OneDoFJointReadOnly> nameToOneDoFJointMap = controllerInput.getInput().getAllJoints().stream().filter(OneDoFJointReadOnly.class::isInstance)
                                                                             .map(joint -> (OneDoFJointReadOnly) joint)
                                                                             .collect(Collectors.toMap(j -> j.getName(), Function.identity()));

      OneDoFJointReadOnly[] jointsToControl = new OneDoFJointReadOnly[namesOfJointsToControl.size()];

      for (int i = 0; i < namesOfJointsToControl.size(); i++)
      {
         String jointName = namesOfJointsToControl.get(i);
         OneDoFJointReadOnly jointToControl = nameToOneDoFJointMap.get(jointName);

         if (jointToControl == null)
            throw new IllegalArgumentException("No 1-DoF joint corresponds to the name: " + jointName + ", this contorller only support 1-DoF joints.");

         jointsToControl[i] = jointToControl;
      }

      OneDoFJointStateBasics[] jointOutputs = controllerOutput.getOneDoFJointOutputs(jointsToControl);
      return new OneDoFJointDampingController(controllerName, damping, jointsToControl, jointOutputs, controllerRegistry);
   }
}
