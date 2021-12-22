package us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem;

import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.JavaFXTools;

public class FrameNode
{
   private final ReferenceFrame referenceFrame;
   private final Node node;
   private final Affine nodePose = new Affine();

   public FrameNode(ReferenceFrame referenceFrame, Node node)
   {
      this.referenceFrame = referenceFrame;
      this.node = node;
      node.getTransforms().add(0, nodePose);
   }

   public void updatePose()
   {
      nodePose.setToTransform(JavaFXTools.createRigidBodyTransformToAffine(referenceFrame.getTransformToRoot()));
   }

   public Node getNode()
   {
      return node;
   }
}
