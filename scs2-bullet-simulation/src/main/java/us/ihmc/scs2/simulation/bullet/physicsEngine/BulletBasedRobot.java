package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletBasedRobot extends RobotExtension 
{
   private final BulletBasedRobotPhysics robotPhysics;
   Robot robot;
   //private btRigidBody bulletRigidBody;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   //private final List<Pair<SimJointBasics, Object>> jointPairingList = new ArrayList<>();
   private final btMotionState bulletMotionState = new btMotionState()
   {
      @Override
      public void setWorldTransform(Matrix4 transformToWorld)
      {
         copyBulletTransformToThis(transformToWorld);
      }

      @Override
      public void getWorldTransform(Matrix4 transformToWorld)
      {
         getThisTransformForCopyToBullet(transformToWorld);
      }
   };
   
   public void copyBulletTransformToThis(Matrix4 transformToWorld)
   {
      BulletTools.toEuclid(transformToWorld, tempTransform);
      SixDoFJointBasics rootJoint = (SixDoFJointBasics) robot.getRootBody().getChildrenJoints().get(0);
      rootJoint.getJointPose().set(tempTransform);
   }
   
   public void getThisTransformForCopyToBullet(Matrix4 transformToWorld)
   {
      SixDoFJointBasics rootJoint = (SixDoFJointBasics) robot.getRootBody().getChildrenJoints().get(0);
      rootJoint.getJointPose().get(tempTransform);
      BulletTools.toBullet(tempTransform, transformToWorld);
   }
   
   public BulletBasedRobot(Robot robot, YoRegistry physicsRegistry)
   {
      super(robot, physicsRegistry);
      robotPhysics = new BulletBasedRobotPhysics(this);
      createBulletPhysicsFrom(robot);
      this.robot = robot;
   }

   private void createBulletPhysicsFrom(Robot robot)
   {
      robot.getRobotDefinition();
      // Instantiate the bullet physics objects.
      // Initialize the jointPairingList.
   }

   public void updateSensors()
   {
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
         joint.getAuxialiryData().update(robotPhysics.getPhysicsOutput());
      }
   }
   
   public btMotionState getBulletMotionState()
   {
      return bulletMotionState;
   }
}
