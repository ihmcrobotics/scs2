package us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem;

import java.util.List;
import java.util.Map;
import java.util.concurrent.Executor;
import java.util.function.BiConsumer;

import javafx.scene.Node;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.CrossFourBarJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteTwinsJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.scs2.definition.robot.CrossFourBarJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteTwinsJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.definition.JavaFXVisualTools;

public class RigidBodyFrameNodeFactories
{
   public static void createRobotFrameNodeMap(RigidBodyReadOnly rootBody,
                                              RobotDefinition robotDefinition,
                                              Executor graphicLoader,
                                              Map<String, FrameNode> frameNodesToPack)
   {
      BiConsumer<String, FrameNode> frameNodeAddition = (bodyName, frameNode) ->
      {
         if (frameNode == null)
            return;
         if (graphicLoader != null)
            graphicLoader.execute(() -> frameNodesToPack.put(bodyName, frameNode));
         else
            frameNodesToPack.put(bodyName, frameNode);
      };

      for (RigidBodyReadOnly body : rootBody.subtreeIterable())
      {
         frameNodeAddition.accept(body.getName(), loadRigidBodyGraphic(robotDefinition, body));

         if (body.getParentJoint() != null)
         {
            if (body.getParentJoint() instanceof CrossFourBarJointReadOnly)
            {
               CrossFourBarJointReadOnly parentJoint = (CrossFourBarJointReadOnly) body.getParentJoint();
               CrossFourBarJointDefinition parentJointDefinition = (CrossFourBarJointDefinition) robotDefinition.getJointDefinition(parentJoint.getName());
               frameNodeAddition.accept(parentJoint.getBodyDA().getName(),
                                        loadRigidBodyGraphic(parentJointDefinition.getBodyDA(),
                                                             parentJoint.getBodyDA(),
                                                             robotDefinition.getResourceClassLoader()));
               frameNodeAddition.accept(parentJoint.getBodyBC().getName(),
                                        loadRigidBodyGraphic(parentJointDefinition.getBodyBC(),
                                                             parentJoint.getBodyBC(),
                                                             robotDefinition.getResourceClassLoader()));
            }
            else if (body.getParentJoint() instanceof RevoluteTwinsJointReadOnly)
            {
               RevoluteTwinsJointReadOnly parentJoint = (RevoluteTwinsJointReadOnly) body.getParentJoint();
               RevoluteTwinsJointDefinition parentJointDefinition = (RevoluteTwinsJointDefinition) robotDefinition.getJointDefinition(parentJoint.getName());
               frameNodeAddition.accept(parentJoint.getBodyAB().getName(),
                                        loadRigidBodyGraphic(parentJointDefinition.getBodyAB(),
                                                             parentJoint.getBodyAB(),
                                                             robotDefinition.getResourceClassLoader()));
            }
         }
      }
   }

   private static FrameNode loadRigidBodyGraphic(RobotDefinition robotDefinition, RigidBodyReadOnly rigidBody)
   {
      return loadRigidBodyGraphic(robotDefinition.getRigidBodyDefinition(rigidBody.getName()), rigidBody, robotDefinition.getResourceClassLoader());
   }

   private static FrameNode loadRigidBodyGraphic(RigidBodyDefinition rigidBodyDefinition, RigidBodyReadOnly rigidBody, ClassLoader resourceClassLoader)
   {
      if (rigidBodyDefinition == null)
         return null;
      else
         return loadRigidBodyGraphic(rigidBodyDefinition.getVisualDefinitions(), rigidBody, resourceClassLoader);
   }

   private static FrameNode loadRigidBodyGraphic(List<VisualDefinition> visualDefinitions, RigidBodyReadOnly rigidBody, ClassLoader resourceClassLoader)
   {
      if (visualDefinitions == null || visualDefinitions.isEmpty())
         return null;

      Node graphicNode = JavaFXVisualTools.collectNodes(visualDefinitions, resourceClassLoader);

      if (graphicNode == null)
         return null;

      ReferenceFrame graphicFrame = rigidBody.isRootBody() ? rigidBody.getBodyFixedFrame() : rigidBody.getParentJoint().getFrameAfterJoint();
      return new FrameNode(graphicFrame, graphicNode);
   }
}
