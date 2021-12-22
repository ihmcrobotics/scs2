package us.ihmc.scs2.session;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFixedReferenceFrameUsingYawPitchRoll extends ReferenceFrame
{
   private final YoFramePoseUsingYawPitchRoll offset;

   public YoFixedReferenceFrameUsingYawPitchRoll(String frameName, String variableNamePrefix, ReferenceFrame parentFrame, YoRegistry registry)
   {
      this(frameName, new YoFramePoseUsingYawPitchRoll(variableNamePrefix, parentFrame, registry), parentFrame);
   }

   public YoFixedReferenceFrameUsingYawPitchRoll(String frameName, YoFramePoseUsingYawPitchRoll offset, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, new RigidBodyTransform(), parentFrame.isAStationaryFrame(), false, true);

      offset.checkReferenceFrameMatch(parentFrame);
      this.offset = offset;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      offset.get(transformToParent);
   }

   public YoFramePoseUsingYawPitchRoll getOffset()
   {
      return offset;
   }
}
