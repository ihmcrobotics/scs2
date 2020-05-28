package us.ihmc.scs2;

import java.util.List;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.JointBuilder;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.mecano.yoVariables.tools.YoMultiBodySystemFactories;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.protobuf.ThreeProto.MeshGroup;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class MultiBodySystemFactoriesJS
{
   public static RigidBodyJS toMultiBodySystemJS(RigidBodyReadOnly originalRootBody, ReferenceFrame cloneStationaryFrame, RobotDefinition robotDefinition)
   {
      return toMultiBodySystemJS(originalRootBody, cloneStationaryFrame, robotDefinition, MultiBodySystemFactories.DEFAULT_JOINT_BUILDER);
   }

   public static RigidBodyJS toYoMultiBodySystemJS(RigidBodyReadOnly originalRootBody, ReferenceFrame cloneStationaryFrame, RobotDefinition robotDefinition,
                                                   YoVariableRegistry registry)
   {
      return toMultiBodySystemJS(originalRootBody, cloneStationaryFrame, robotDefinition, YoMultiBodySystemFactories.newYoJointBuilder(registry));
   }

   public static RigidBodyJS toMultiBodySystemJS(RigidBodyReadOnly originalRootBody, ReferenceFrame cloneStationaryFrame, RobotDefinition robotDefinition,
                                                 JointBuilder jointBuilder)
   {
      return (RigidBodyJS) MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody,
                                                                         cloneStationaryFrame,
                                                                         "",
                                                                         newRigidBodyJSBuilder(robotDefinition),
                                                                         jointBuilder);
   }

   public static RigidBodyBuilder newRigidBodyJSBuilder(RobotDefinition robotDefinition)
   {
      return newRigidBodyJSBuilder(MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER, robotDefinition);
   }

   public static RigidBodyBuilder newRigidBodyJSBuilder(RigidBodyBuilder rigidBodyBuilder, RobotDefinition robotDefinition)
   {
      return new RigidBodyBuilder()
      {
         @Override
         public RigidBodyJS buildRoot(String bodyName, RigidBodyTransform transformToParent, ReferenceFrame parentStationaryFrame)
         {
            RigidBodyBasics rootBody = rigidBodyBuilder.buildRoot(bodyName, transformToParent, parentStationaryFrame);
            return toRigidBodyJS(rootBody, robotDefinition.getRigidBodyDefinition(rootBody.getName()));
         }

         @Override
         public RigidBodyJS build(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransform inertiaPose)
         {
            RigidBodyBasics rigidBody = rigidBodyBuilder.build(bodyName, parentJoint, momentOfInertia, mass, inertiaPose);
            return toRigidBodyJS(rigidBody, robotDefinition.getRigidBodyDefinition(rigidBody.getName()));
         }
      };
   }

   public static RigidBodyJS toRigidBodyJS(RigidBodyBasics rigidBody, RigidBodyDefinition rigidBodyDefinition)
   {
      RigidBodyJS rigidBodyJS = new RigidBodyJS(rigidBody);
      loadRigidBodyGraphic(rigidBodyDefinition.getVisualDefinitions(), rigidBodyJS);
      return rigidBodyJS;
   }

   private static void loadRigidBodyGraphic(List<VisualDefinition> visualDefinitions, RigidBodyJS rigidBodyJS)
   {
      MeshGroup meshGroup = ThreeProtoTools.toProtoMeshGroup(rigidBodyJS.getName(), visualDefinitions);
      ReferenceFrame graphicFrame = rigidBodyJS.isRootBody() ? rigidBodyJS.getBodyFixedFrame() : rigidBodyJS.getParentJoint().getFrameAfterJoint();

      if (meshGroup != null)
         rigidBodyJS.setGraphics(new FrameMeshGroupJS(graphicFrame, meshGroup));
   }
}
