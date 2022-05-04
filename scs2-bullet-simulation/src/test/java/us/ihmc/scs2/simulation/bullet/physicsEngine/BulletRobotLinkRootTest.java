package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.Random;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;

class BulletRobotLinkRootTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1e-5;

   @Test
   public void testAccelerationCalculation()
   {
      Random random = new Random(4354353);

      RigidBody rootBody = new RigidBody("root", ReferenceFrame.getWorldFrame());
      SixDoFJoint joint = MultiBodySystemRandomTools.nextSixDoFJoint(random, "joint", rootBody);
      MovingReferenceFrame frameAfter = joint.getFrameAfterJoint();
      MovingReferenceFrame frameBefore = joint.getFrameBeforeJoint();
      Pose3D finalPose = new Pose3D();
      Twist finalTwist = new Twist(frameAfter, frameBefore, frameAfter);
      SpatialAcceleration finalAcceleration = new SpatialAcceleration(frameAfter, frameBefore, frameAfter);
      SpatialAcceleration estimatedAcceleration = new SpatialAcceleration(frameAfter, frameBefore, frameAfter);

      double dt = 1.0e-5;
      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator(dt);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Pose3D initialPose = EuclidGeometryRandomTools.nextPose3D(random);
         Twist initialTwist = MecanoRandomTools.nextTwist(random, frameAfter, frameBefore, frameAfter);
         SpatialAcceleration initialAcceleration = MecanoRandomTools.nextSpatialAcceleration(random, frameAfter, frameBefore, frameAfter);
         joint.getJointPose().set(initialPose);
         joint.getJointTwist().set(initialTwist);
         joint.getJointAcceleration().set(initialAcceleration);
         joint.updateFrame();

         finalPose.set(initialPose);
         finalTwist.set(initialTwist);
         finalAcceleration.set(initialAcceleration);
         integrator.doubleIntegrate(finalAcceleration, finalTwist, finalPose);
         joint.updateFrame();

         BulletRobotLinkRoot.computeJointAcceleration(dt, initialPose, finalPose, initialTwist, finalTwist, estimatedAcceleration);
         MecanoTestTools.assertSpatialAccelerationEquals("Iteration: " + i, finalAcceleration, estimatedAcceleration, EPSILON);
      }
   }

}
