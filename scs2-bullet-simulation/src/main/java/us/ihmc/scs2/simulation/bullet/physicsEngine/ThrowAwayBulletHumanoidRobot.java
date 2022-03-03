package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

public class ThrowAwayBulletHumanoidRobot
{
   private final RobotDefinition robotDefinition;
   private final RigidBodyBasics rootRigidBody;
   private final ThrowAwayBulletRobotLink elevator;
   private final ThrowAwayBulletRobotLink pelvisLink;
   private final float pelvisLinkMass;
   private final SixDoFJoint pelvisSixDoFJoint;
   private int linkCountingIndex = 0;
   private int numberOfLinks;
   private final HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<>();
   private final ArrayList<ThrowAwayBulletRobotLink> allLinks = new ArrayList<>();
   private final ArrayList<ThrowAwayBulletRobotLink> afterElevatorLinks = new ArrayList<>();
   private final ArrayList<ThrowAwayBulletRobotLink> afterPelvisLinks = new ArrayList<>();
   private btMultiBody bulletMultiBody;
   private btMultiBodyLinkCollider baseCollider;
   private final Matrix4 bulletPelvisColliderCenterOfMassTransformToWorldGDX = new Matrix4();
   private final RigidBodyTransform bulletPelvisColliderCenterOfMassTransformToWorldEuclid = new RigidBodyTransform();
   private final RigidBodyTransform bulletPelvisAfterJointTransformToWorldEuclid = new RigidBodyTransform();
   private final RigidBodyTransform bulletPelvisCenterOfMassTransformToAfterParentJointEuclid = new RigidBodyTransform();
   private final YoRegistry yoRegistry;

   public ThrowAwayBulletHumanoidRobot(RobotDefinition robotDefinition)
   {
      this.robotDefinition = robotDefinition;
      yoRegistry = new YoRegistry(robotDefinition.getName() + getClass().getSimpleName());
      rootRigidBody = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
      rootRigidBody.updateFramesRecursively();
      JointDefinition pelvisSixDoFJointDefinition = robotDefinition.getRootJointDefinitions().get(0);
      numberOfLinks = countJoints(pelvisSixDoFJointDefinition) - 1; // which is also number of joints in this case
      elevator = new ThrowAwayBulletRobotLink(robotDefinition.getRootBodyDefinition(), rootRigidBody, jointNameToBulletJointIndexMap, yoRegistry);
      initializeLinkLists(elevator);
      pelvisLink = elevator.getChildren().get(0);
      pelvisLinkMass = (float) pelvisLink.getRigidBodyDefinition().getMass();
      pelvisSixDoFJoint = (SixDoFJoint) pelvisLink.getParentJoint();
      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.set(pelvisLink.getRigidBodyDefinition().getInertiaPose());
   }

   private void initializeLinkLists(ThrowAwayBulletRobotLink link)
   {
      boolean isElevator = link.getRigidBodyDefinition().getName().equals("elevator");
      boolean isPelvis = link.getRigidBodyDefinition().getName().equals("pelvis");
      allLinks.add(link);
      if (!isElevator)
      {
         afterElevatorLinks.add(link);
      }
      if (!isElevator && !isPelvis)
      {
         afterPelvisLinks.add(link);
      }

      for (ThrowAwayBulletRobotLink child : link.getChildren())
      {
         initializeLinkLists(child);
      }
   }

   private int countJoints(JointDefinition joint)
   {
      jointNameToBulletJointIndexMap.put(joint.getName(), linkCountingIndex - 1);
      ++linkCountingIndex;
      int numberOfJoints = 1;
      for (JointDefinition childrenJoint : joint.getSuccessor().getChildrenJoints())
      {
         numberOfJoints += countJoints(childrenJoint);
      }
      return numberOfJoints;
   }

   public void createBulletMultiBody(Consumer<btMultiBodyLinkCollider> colliderRegistration)
   {
      boolean fixedBase = false;
      boolean canSleep = false;

      Vector3 pelvisIntertia = new Vector3((float) pelvisLink.getRigidBodyDefinition().getMomentOfInertia().getM00(),
                                           (float) pelvisLink.getRigidBodyDefinition().getMomentOfInertia().getM11(),
                                           (float) pelvisLink.getRigidBodyDefinition().getMomentOfInertia().getM22());
      bulletMultiBody = new btMultiBody(numberOfLinks, pelvisLinkMass, pelvisIntertia, fixedBase, canSleep);
      bulletMultiBody.setHasSelfCollision(true);
      bulletMultiBody.setUseGyroTerm(true);
      bulletMultiBody.setLinearDamping(0.1f);
      bulletMultiBody.setAngularDamping(0.9f);

      pelvisLink.createBulletCollisionShape(bulletMultiBody, colliderRegistration);
      baseCollider = pelvisLink.getBulletMultiBodyLinkCollider();
      bulletMultiBody.setBaseCollider(baseCollider);

      for (ThrowAwayBulletRobotLink link : afterPelvisLinks)
      {
         link.createBulletJoint(bulletMultiBody);
         link.createBulletCollisionShape(bulletMultiBody, colliderRegistration);
      }
      bulletMultiBody.finalizeMultiDof();
   }

   public void setPelvisCenterOfMassTransformToWorld(RigidBodyTransform pelvisCenterOfMassTransformToWorld)
   {
      BulletTools.toBullet(pelvisCenterOfMassTransformToWorld, bulletPelvisColliderCenterOfMassTransformToWorldGDX);
      bulletMultiBody.setBaseWorldTransform(bulletPelvisColliderCenterOfMassTransformToWorldGDX);
      baseCollider.setWorldTransform(bulletPelvisColliderCenterOfMassTransformToWorldGDX);
      bulletPelvisAfterJointTransformToWorldEuclid.set(pelvisCenterOfMassTransformToWorld);
      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.inverseTransform(bulletPelvisAfterJointTransformToWorldEuclid);
      pelvisSixDoFJoint.setJointPosition(bulletPelvisAfterJointTransformToWorldEuclid.getTranslation());
      pelvisSixDoFJoint.setJointOrientation(bulletPelvisAfterJointTransformToWorldEuclid.getRotation());
      rootRigidBody.updateFramesRecursively();
   }

   public void initializeRobotToDefaultJointAngles()
   {
      for (JointBasics joint : rootRigidBody.childrenSubtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            robotDefinition.getJointDefinition(joint.getName()).getInitialJointState().getConfiguration(joint);
         }
      }
      rootRigidBody.updateFramesRecursively();
   }

   public void updateFromMecanoRigidBody()
   {
      updateFromMecanoRigidBody(pelvisLink);
   }

   private void updateFromMecanoRigidBody(ThrowAwayBulletRobotLink link)
   {
      link.updateFromMecanoRigidBody();

      for (ThrowAwayBulletRobotLink child : link.getChildren())
      {
         updateFromMecanoRigidBody(child);
      }
   }

   public void updateFromBulletData()
   {
      baseCollider.getWorldTransform(bulletPelvisColliderCenterOfMassTransformToWorldGDX);
      BulletTools.toEuclid(bulletPelvisColliderCenterOfMassTransformToWorldGDX, bulletPelvisColliderCenterOfMassTransformToWorldEuclid);
      bulletPelvisAfterJointTransformToWorldEuclid.set(bulletPelvisColliderCenterOfMassTransformToWorldEuclid);
      bulletPelvisCenterOfMassTransformToAfterParentJointEuclid.inverseTransform(bulletPelvisAfterJointTransformToWorldEuclid);
      pelvisSixDoFJoint.setJointPosition(bulletPelvisAfterJointTransformToWorldEuclid.getTranslation());
      pelvisSixDoFJoint.setJointOrientation(bulletPelvisAfterJointTransformToWorldEuclid.getRotation());
      // TODO: Calculate velocity & acceleration to pack Mecano stuff?

      for (ThrowAwayBulletRobotLink afterPelvisLink : afterPelvisLinks)
      {
         afterPelvisLink.updateJointAngleFromBulletData();
      }

      rootRigidBody.updateFramesRecursively();

      for (ThrowAwayBulletRobotLink afterElevatorLink : afterElevatorLinks)
      {
         afterElevatorLink.updateFrames();
      }
   }

   public void afterSimulate()
   {
      for (ThrowAwayBulletRobotLink afterPelvisLink : afterPelvisLinks)
      {
         afterPelvisLink.afterSimulate();
      }
   }

   public ThrowAwayBulletRobotLink getElevator()
   {
      return elevator;
   }

   public ThrowAwayBulletRobotLink getPelvisLink()
   {
      return pelvisLink;
   }

   public btMultiBody getBulletMultiBody()
   {
      return bulletMultiBody;
   }

   public float getPelvisLinkMass()
   {
      return pelvisLinkMass;
   }

   public ArrayList<ThrowAwayBulletRobotLink> getAfterPelvisLinks()
   {
      return afterPelvisLinks;
   }

   public ArrayList<ThrowAwayBulletRobotLink> getAfterElevatorLinks()
   {
      return afterElevatorLinks;
   }

   public ArrayList<ThrowAwayBulletRobotLink> getAllLinks()
   {
      return allLinks;
   }
}
