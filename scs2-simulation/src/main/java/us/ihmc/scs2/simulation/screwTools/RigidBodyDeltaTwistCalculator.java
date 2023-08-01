package us.ihmc.scs2.simulation.screwTools;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

public class RigidBodyDeltaTwistCalculator implements Function<RigidBodyReadOnly, TwistReadOnly>
{
   private final ReferenceFrame inertialFrame;
   private final JointMatrixIndexProvider jointMatrixIndexProvider;
   private RigidBodyTwistProvider deltaTwistProvider;
   private final DMatrixRMaj velocityChangeMatrix;

   public RigidBodyDeltaTwistCalculator(ReferenceFrame inertialFrame, JointMatrixIndexProvider jointMatrixIndexProvider, DMatrixRMaj velocityChangeMatrix)
   {
      this.inertialFrame = inertialFrame;
      this.jointMatrixIndexProvider = jointMatrixIndexProvider;
      this.velocityChangeMatrix = velocityChangeMatrix;
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

   private final Map<RigidBodyReadOnly, Twist> rigidBodyTwistMap = new HashMap<>();
   private final Twist jointTwist = new Twist();

   @Override
   public TwistReadOnly apply(RigidBodyReadOnly body)
   {
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

         // TODO Implements other joints
         if (parentJoint instanceof OneDoFJointReadOnly)
         {
            jointTwist.setIncludingFrame(((OneDoFJointReadOnly) parentJoint).getUnitJointTwist());
            jointTwist.scale(velocityChangeMatrix.get(jointMatrixIndexProvider.getJointDoFIndices(parentJoint)[0]));
         }
         else if (parentJoint instanceof SixDoFJointReadOnly)
         {
            jointTwist.set(jointMatrixIndexProvider.getJointDoFIndices(parentJoint)[0], velocityChangeMatrix);
            jointTwist.setReferenceFrame(parentJoint.getFrameAfterJoint());
            jointTwist.setBaseFrame(parentJoint.getFrameBeforeJoint());
            jointTwist.setBodyFrame(parentJoint.getFrameAfterJoint());
         }
         else if (parentJoint instanceof FixedJointReadOnly)
         {
            jointTwist.setToZero(body.getBodyFixedFrame());
         }
         else
         {
            throw new UnsupportedOperationException("Implement me for: " + parentJoint.getClass().getSimpleName());
         }

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
