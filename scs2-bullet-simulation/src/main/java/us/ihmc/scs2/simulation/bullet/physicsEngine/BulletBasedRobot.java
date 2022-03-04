package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletBasedRobot extends RobotExtension 
{
   private final BulletBasedRobotPhysics robotPhysics;
   private final RigidBodyDefinition rootBodyDefinition;
   private final SimRigidBodyBasics rootSimRigidBodyBasics;
   private final SimFloatingRootJoint rootSimFloatingRootJoint;
   private final BulletBasedRobotLinkRoot rootLink;
   private final HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<>();
   private final ArrayList<BulletBasedRobotLinkBasics> allLinks = new ArrayList<>();
   private final ArrayList<BulletBasedRobotLinkRevolute> afterRootLinks = new ArrayList<>();
   private final YoRegistry yoRegistry;
   
   public BulletBasedRobot(Robot robot, YoRegistry physicsRegistry, BulletBasedPhysicsEngine bulletPhysicsManager)
   {
      super(robot, physicsRegistry);
      robotPhysics = new BulletBasedRobotPhysics(this);

      // Initialize the jointPairingList?
      
      yoRegistry = new YoRegistry(getRobotDefinition().getName() + getClass().getSimpleName());

      rootBodyDefinition = robot.getRobotDefinition().getRootBodyDefinition();
      JointDefinition rootJointDefinition = rootBodyDefinition.getChildrenJoints().get(0);
      if (!(rootJointDefinition instanceof SixDoFJointDefinition))
         throw new RuntimeException("Expecting a SixDoFJointDefinition, not a " + rootJointDefinition.getClass().getSimpleName());
      SixDoFJointDefinition rootSixDoFJointDefinition = (SixDoFJointDefinition) rootJointDefinition;

      rootSimRigidBodyBasics = robot.getRootBody();
      JointBasics rootJoint = rootSimRigidBodyBasics.getChildrenJoints().get(0);
      if (!(rootJoint instanceof SimFloatingRootJoint))
         throw new RuntimeException("Expecting a SimFloatingRootJoint, not a " + rootJoint.getClass().getSimpleName());
      rootSimFloatingRootJoint = (SimFloatingRootJoint) rootJoint;
      rootLink = new BulletBasedRobotLinkRoot(rootSixDoFJointDefinition, rootSimFloatingRootJoint, jointNameToBulletJointIndexMap, yoRegistry);
      initializeLinkLists(rootLink, true);

      rootLink.setup();
      rootLink.createBulletCollisionShape(bulletPhysicsManager);

      for (BulletBasedRobotLinkRevolute link : afterRootLinks)
      {
         link.setBulletMultiBody(rootLink.getBulletMultiBody());
         link.setup();
         link.createBulletCollisionShape(bulletPhysicsManager);
      }
      rootLink.getBulletMultiBody().finalizeMultiDof();
   }
   
   private void initializeLinkLists(BulletBasedRobotLinkBasics link, boolean isRootLink)
   {
      allLinks.add(link);
      if (!isRootLink)
      {
         afterRootLinks.add((BulletBasedRobotLinkRevolute) link);
      }

      for (BulletBasedRobotLinkBasics child : link.getChildren())
      {
         initializeLinkLists(child, false);
      }
   }

   @Override
   public void initializeState()
   {
      super.initializeState();

      copyDataFromSCSToBullet();
   }

   @Override
   public void saveRobotBeforePhysicsState()
   {
      copyDataFromSCSToBullet();
   }

   public void copyDataFromSCSToBullet()
   {
      copyDataFromSCSToBullet(rootLink);
   }

   private void copyDataFromSCSToBullet(BulletBasedRobotLinkBasics link)
   {
      link.updateBulletLinkColliderTransformFromMecanoRigidBody();

      for (BulletBasedRobotLinkBasics child : link.getChildren())
      {
         copyDataFromSCSToBullet(child);
      }
   }
   
   public void updateFromBulletData()
   {
      rootLink.copyBulletJointDataToSCS();

      for (BulletBasedRobotLinkRevolute afterPelvisLink : afterRootLinks)
      {
         afterPelvisLink.copyBulletJointDataToSCS();
      }
   }

   public void afterSimulate()
   {
      for (BulletBasedRobotLinkRevolute afterPelvisLink : afterRootLinks)
      {
         afterPelvisLink.afterSimulate();
      }
   }

   public BulletBasedRobotLinkRoot getRootLink()
   {
      return rootLink;
   }

   public btMultiBody getBulletMultiBody()
   {
      return rootLink.getBulletMultiBody();
   }

   public ArrayList<BulletBasedRobotLinkRevolute> getAfterRootLinks()
   {
      return afterRootLinks;
   }

   public ArrayList<BulletBasedRobotLinkBasics> getAllLinks()
   {
      return allLinks;
   }

   public void updateSensors()
   {
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
         joint.getAuxialiryData().update(robotPhysics.getPhysicsOutput());
      }
   }
}
