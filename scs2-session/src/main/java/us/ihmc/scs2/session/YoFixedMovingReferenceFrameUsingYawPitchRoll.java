package us.ihmc.scs2.session;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFixedMovingReferenceFrameUsingYawPitchRoll extends MovingReferenceFrame
{
   private final YoFramePoseUsingYawPitchRoll offset;

   public YoFixedMovingReferenceFrameUsingYawPitchRoll(String frameName, String variableNamePrefix, ReferenceFrame parentFrame, YoRegistry registry)
   {
      this(frameName, new YoFramePoseUsingYawPitchRoll(variableNamePrefix, parentFrame, registry), parentFrame);
   }

   public YoFixedMovingReferenceFrameUsingYawPitchRoll(String frameName, YoFramePoseUsingYawPitchRoll offset, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, new RigidBodyTransform(), false, true);

      offset.checkReferenceFrameMatch(parentFrame);
      this.offset = offset;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      offset.get(transformToParent);
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
   }

   public YoFramePoseUsingYawPitchRoll getOffset()
   {
      return offset;
   }
}
