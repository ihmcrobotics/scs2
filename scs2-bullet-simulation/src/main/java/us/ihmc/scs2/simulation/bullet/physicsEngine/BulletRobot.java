package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletRobot extends RobotExtension
{
   private final BulletRobotPhysics robotPhysics;
   private SimFloatingRootJoint rootSimFloatingRootJoint;
   private final BulletMultiBodyRobot bulletMultiBodyRobot;
   private BulletRobotLinkRoot rootLink;
   private final ArrayList<BulletRobotLinkJoint> afterRootLinks = new ArrayList<>();
   private final YoRegistry yoRegistry;

   public BulletRobot(Robot robot, YoRegistry physicsRegistry, BulletMultiBodyRobot bulletMultiBodyRobot)
   {
      super(robot, physicsRegistry);
      robotPhysics = new BulletRobotPhysics(this);

      yoRegistry = new YoRegistry(getRobotDefinition().getName() + getClass().getSimpleName());
      robot.getRegistry().addChild(yoRegistry);
      this.bulletMultiBodyRobot = bulletMultiBodyRobot;
      
      for (BulletMultiBodyLinkCollider bulletLinkCollider : bulletMultiBodyRobot.getBulletMultiBodyLinkColliderArray())
      {
         int bulletJointIndex = bulletMultiBodyRobot.getJointNameToBulletJointIndexMap().get(bulletLinkCollider.getJointName());
      
         JointBasics joint = robot.getJoint(bulletLinkCollider.getJointName());
         
         if (bulletJointIndex == -1)
         {
            if (!(joint instanceof SimFloatingRootJoint))
               throw new RuntimeException("Expecting a SimFloatingRootJoint, not a " + joint.getClass().getSimpleName());
            
            rootSimFloatingRootJoint = (SimFloatingRootJoint) joint;
            rootLink = new BulletRobotLinkRoot(rootSimFloatingRootJoint,
                                               robotPhysics.getRigidBodyWrenchRegistry(),
                                               yoRegistry,
                                               bulletLinkCollider);
         } else { 
            if (joint instanceof SimOneDoFJointBasics)
            {
               SimOneDoFJointBasics childSimOneDoFJoint = (SimOneDoFJointBasics) joint;
               afterRootLinks.add(new BulletRobotLinkJoint(childSimOneDoFJoint,
                                                           bulletJointIndex,
                                                           robotPhysics.getRigidBodyWrenchRegistry(),
                                                           yoRegistry,
                                                           bulletLinkCollider));
            }
         }
      }
   }

   public void copyDataFromSCSToBullet()
   {
      robotPhysics.reset();

      if (rootLink != null)
      rootLink.copyDataFromSCSToBullet();

      for (BulletRobotLinkJoint afterRootLink : afterRootLinks)
      {
         afterRootLink.copyDataFromSCSToBullet();
      }
   }

   public void updateFromBulletData(BulletPhysicsEngine bulletPhysicsEngine, double dt)
   {
      if (rootLink != null)
      rootLink.copyBulletJointDataToSCS(dt);

      for (BulletRobotLinkJoint afterRootLink : afterRootLinks)
      {
         afterRootLink.copyBulletJointDataToSCS(dt);
      }
      robotPhysics.update();
   }
   
   public BulletMultiBodyRobot getBulletMultiBodyRobot()
   {
      return bulletMultiBodyRobot;
   }

   public void updateSensors()
   {
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
         joint.getAuxialiryData().update(robotPhysics.getPhysicsOutput());
      }
   }
}
