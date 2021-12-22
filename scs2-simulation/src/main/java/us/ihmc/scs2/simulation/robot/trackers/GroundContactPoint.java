package us.ihmc.scs2.simulation.robot.trackers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class GroundContactPoint extends ExternalWrenchPoint
{
   private final YoFramePose3D touchdownPose;
   private final YoFrameVector3D contactNormal;

   private final YoBoolean inContact;
   private final YoBoolean isSlipping;

   public GroundContactPoint(KinematicPointDefinition definition, SimJointBasics parentJoint)
   {
      this(definition.getName(), parentJoint, definition.getTransformToParent());
   }

   public GroundContactPoint(String name, SimJointBasics parentJoint, RigidBodyTransformReadOnly transformToParent)
   {
      super(name, parentJoint, transformToParent);

      ReferenceFrame rootFrame = parentJoint.getFrameAfterJoint().getRootFrame();
      YoRegistry registry = parentJoint.getRegistry();
      touchdownPose = new YoFramePose3D(name + "Touchdown", rootFrame, registry);
      contactNormal = new YoFrameVector3D(name + "ContactNormal", rootFrame, registry);
      inContact = new YoBoolean(name + "InContact", registry);
      isSlipping = new YoBoolean(name + "IsSlipping", registry);
   }

   public YoFramePose3D getTouchdownPose()
   {
      return touchdownPose;
   }

   public YoFrameVector3D getContactNormal()
   {
      return contactNormal;
   }

   public YoBoolean getInContact()
   {
      return inContact;
   }

   public YoBoolean getIsSlipping()
   {
      return isSlipping;
   }
}
