package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.linearmath.btMotionState;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletBasedRobot extends RobotExtension 
{
   private final BulletBasedRobotPhysics robotPhysics;
   Robot robot;
   btMultiBody bulletMultiBodyRobot;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private Matrix4 tempTransformToWorld = new Matrix4();
   //private final List<Pair<SimJointBasics, Object>> jointPairingList = new ArrayList<>();
   private final btMotionState bulletMotionState = new btMotionState()
   {
      @Override
      public void setWorldTransform(Matrix4 transformToWorld)
      {
         BulletTools.toEuclid(transformToWorld, tempTransform);
         SixDoFJointBasics rootJoint = (SixDoFJointBasics) robot.getRootBody().getChildrenJoints().get(0);
         rootJoint.getJointPose().set(tempTransform);
      }

      @Override
      public void getWorldTransform(Matrix4 transformToWorld)
      {
         SixDoFJointBasics rootJoint = (SixDoFJointBasics) robot.getRootBody().getChildrenJoints().get(0);
         rootJoint.getJointPose().get(tempTransform);
         BulletTools.toBullet(tempTransform, transformToWorld);
      }
   };
   
   public BulletBasedRobot(Robot robot, YoRegistry physicsRegistry)
   {
      super(robot, physicsRegistry);
      robotPhysics = new BulletBasedRobotPhysics(this);
      createBulletPhysicsFrom(robot);
      this.robot = robot;
   }
   
   public void getWorldTransformation()
   {
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
         if (joint.getName() != "rootJoint")
         {
            SimRigidBodyBasics successor = joint.getSuccessor();
            SimRigidBodyBasics predecessor = joint.getPredecessor();
           
            BulletTools.toBullet(successor.getBodyFixedFrame().getTransformToRoot(), tempTransformToWorld);
            bulletMultiBodyRobot.setBaseWorldTransform(tempTransformToWorld);
            bulletMultiBodyRobot.getBaseCollider().setWorldTransform(tempTransformToWorld);
            
            BulletTools.toBullet(predecessor.getBodyFixedFrame().getTransformToRoot(), tempTransformToWorld);
            bulletMultiBodyRobot.getLink(0).getCollider().setWorldTransform(tempTransformToWorld);
         }
      }
   }

   public void setWorldTransformation()
   {
//      getRootBody().getChildrenJoints().get(0).setJointConfiguration(tempTransform);
      
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
         if (joint.getName() != "rootJoint")
         {
            SimRigidBodyBasics successor = joint.getSuccessor();
            SimRigidBodyBasics predecessor = joint.getPredecessor();
            // T^successor_frameAfterJoint
            RigidBodyTransform T_succ_afterJoint = joint.getFrameAfterJoint().getTransformToDesiredFrame(successor.getBodyFixedFrame());
            // T^frameBeforeJoint_predecessor
            RigidBodyTransform T_beforeJoint_pred = predecessor.getBodyFixedFrame().getTransformToDesiredFrame(joint.getFrameBeforeJoint());
            
            //tempTransformToWorld = bulletMultiBodyRobot.getBaseCollider().getWorldTransform();
            //BulletTools.toEuclid(tempTransformToWorld, tempTransform);
            
//            System.out.println("setWorld Successor " + successor.getBodyFixedFrame().getTransformToRoot());
//            System.out.println("setWorld Predeccessor " + predecessor.getBodyFixedFrame().getTransformToRoot());

            //tempTransformToWorld = bulletMultiBodyRobot.getLink(0).getCollider().getWorldTransform();
            //BulletTools.toEuclid(tempTransformToWorld, tempTransform);

            //System.out.println("setWorld Base " + bulletMultiBodyRobot.getBaseCollider().getWorldTransform());
           // System.out.println("setWorld link " + bulletMultiBodyRobot.getLink(0).getCollider().getWorldTransform());
            
//            System.out.println("T_succ_afterJoint " + T_succ_afterJoint);
//            System.out.println("T_beforeJoint_pred" + T_beforeJoint_pred);
            
            //joint.getSuccessor().getBodyFixedFrame().getTransformToRoot();
            //joint.getJointConfiguration(tempTransform);
         }
      }
      
      getRootBody().updateFramesRecursively();
      
   }

   
   public void setbtMultiBody(btMultiBody bulletMultiBodyRobot)
   {
      this.bulletMultiBodyRobot = bulletMultiBodyRobot;
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
