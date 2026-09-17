package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;

import org.bytedeco.bullet.BulletCollision.btCollisionObject;
import org.bytedeco.bullet.BulletDynamics.btMultiBody;
import org.bytedeco.bullet.BulletDynamics.btMultiBodyLinkCollider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimOneDoFJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletRobot extends RobotExtension
{
   private final BulletRobotPhysics robotPhysics;
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

            rootLink = new BulletRobotLinkRoot((SimFloatingRootJoint) joint, robotPhysics.getRigidBodyWrenchRegistry(), yoRegistry, bulletLinkCollider);
         }
         else
         {
            if (joint instanceof SimOneDoFJointBasics)
            {
               afterRootLinks.add(new BulletRobotLinkJoint((SimOneDoFJointBasics) joint,
                                                           bulletJointIndex,
                                                           robotPhysics.getRigidBodyWrenchRegistry(),
                                                           yoRegistry,
                                                           bulletLinkCollider));
            }
            else
            {
               throw new RuntimeException("Expecting a SimOneDoFJointBasics, not a " + joint.getClass().getSimpleName());
            }
         }
      }
   }

   public void pushStateToBullet()
   {
      robotPhysics.reset();

      if (rootLink != null)
         rootLink.pushStateToBullet();

      for (BulletRobotLinkJoint afterRootLink : afterRootLinks)
      {
         afterRootLink.pushStateToBullet();
      }
   }

   public void pullStateFromBullet(double dt)
   {
      if (rootLink != null)
         rootLink.pullStateFromBullet(dt);

      for (BulletRobotLinkJoint afterRootLink : afterRootLinks)
      {
         afterRootLink.pullStateFromBullet(dt);
      }
      robotPhysics.update(dt);
   }

   /**
    * Sets the whole robot to be a Bullet kinematic object, which can be moved around
    * and exerts forces on other non-kinematic objects, but is not subject to receiving
    * forces or contacts.
    */
   public void setIsKinematicObject(boolean isKinematicObject)
   {
      btMultiBody btMultiBody = bulletMultiBodyRobot.getBtMultiBody();
      btMultiBodyLinkCollider collider = btMultiBody.getBaseCollider();

      if (isKinematicObject)
         collider.setCollisionFlags(collider.getCollisionFlags() | btCollisionObject.CF_KINEMATIC_OBJECT);
      else
         collider.setCollisionFlags(collider.getCollisionFlags() & ~btCollisionObject.CF_KINEMATIC_OBJECT);

      for (int i = 0; i < btMultiBody.getNumLinks(); i++)
      {
         collider = btMultiBody.getLink(i).m_collider();
         if (!collider.isNull())
         {
            if (isKinematicObject)
               collider.setCollisionFlags(collider.getCollisionFlags() | btCollisionObject.CF_KINEMATIC_OBJECT);
            else
               collider.setCollisionFlags(collider.getCollisionFlags() & ~btCollisionObject.CF_KINEMATIC_OBJECT);
         }
      }
   }

   public BulletMultiBodyRobot getBulletMultiBodyRobot()
   {
      return bulletMultiBodyRobot;
   }

   public void updateSensors()
   {
      getRootBody().updateAuxiliaryDataRecursively(robotPhysics.getPhysicsOutput());
   }
}
