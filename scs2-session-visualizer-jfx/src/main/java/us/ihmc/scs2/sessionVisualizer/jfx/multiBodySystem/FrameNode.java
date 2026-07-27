package us.ihmc.scs2.sessionVisualizer.jfx.multiBodySystem;

import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class FrameNode
{
   private final ReferenceFrame referenceFrame;
   private final Node node;
   private final Affine nodePose = new Affine();

   /**
    * Deep copy of the transform last actually
    */
   private RigidBodyTransform lastAppliedTransform;

   public FrameNode(ReferenceFrame referenceFrame, Node node)
   {
      this.referenceFrame = referenceFrame;
      this.node = node;
      node.getTransforms().add(0, nodePose);
   }

   public void updatePose()
   {
      RigidBodyTransform transformToRoot = referenceFrame.getTransformToRoot();

      if (lastAppliedTransform != null && lastAppliedTransform.equals(transformToRoot))
         return;

      // Write directly into the live nodePose instead of allocating a throwaway Affine via
      // JavaFXMissingTools.createRigidBodyTransformToAffine(...) just to copy it over with setToTransform().
      JavaFXMissingTools.convertRigidBodyTransformToAffine(transformToRoot, nodePose);

      if (lastAppliedTransform == null)
         lastAppliedTransform = new RigidBodyTransform(transformToRoot);
      else
         lastAppliedTransform.set(transformToRoot);
   }

   public Node getNode()
   {
      return node;
   }
}
