package us.ihmc.scs2.simulation.screwTools;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointReadOnly;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyReadOnly;

public class SimRigidBodyDeltaTwistCalculator implements Function<RigidBodyReadOnly, TwistReadOnly>
{
   private final ReferenceFrame inertialFrame;

   private final Map<RigidBodyReadOnly, Twist> rigidBodyTwistMap = new HashMap<>();
   private final Twist jointTwist = new Twist();
   private RigidBodyTwistProvider deltaTwistProvider;

   public SimRigidBodyDeltaTwistCalculator(ReferenceFrame inertialFrame)
   {
      this.inertialFrame = inertialFrame;
      deltaTwistProvider = RigidBodyTwistProvider.toRigidBodyTwistProvider(this, inertialFrame);
   }

   public void reset()
   {
      rigidBodyTwistMap.clear();
   }

   public RigidBodyTwistProvider getDeltaTwistProvider()
   {
      return deltaTwistProvider;
   }

   @Override
   public TwistReadOnly apply(RigidBodyReadOnly body)
   {
      if (body instanceof SimRigidBodyReadOnly)
         throw new IllegalArgumentException("The given body is not a " + SimRigidBodyReadOnly.class.getSimpleName());

      Twist twistOfBody = rigidBodyTwistMap.get(body);

      if (twistOfBody == null)
      {
         JointReadOnly parentJoint = body.getParentJoint();
         TwistReadOnly twistOfParentBody;
         RigidBodyReadOnly parentBody = parentJoint.getPredecessor();
         if (parentBody.isRootBody())
            twistOfParentBody = null;
         else
            twistOfParentBody = apply(parentBody);

         jointTwist.setIncludingFrame(((SimJointReadOnly) parentJoint).getJointDeltaTwist());
         jointTwist.changeFrame(body.getBodyFixedFrame());
         jointTwist.setBaseFrame(parentBody.getBodyFixedFrame());
         jointTwist.setBodyFrame(body.getBodyFixedFrame());

         twistOfBody = new Twist();
         if (twistOfParentBody == null)
            twistOfBody.setToZero(parentBody.getBodyFixedFrame(), inertialFrame, parentBody.getBodyFixedFrame());
         else
            twistOfBody.setIncludingFrame(twistOfParentBody);
         twistOfBody.changeFrame(body.getBodyFixedFrame());
         twistOfBody.add(jointTwist);
         rigidBodyTwistMap.put(body, twistOfBody);
      }

      return twistOfBody;
   }
}
