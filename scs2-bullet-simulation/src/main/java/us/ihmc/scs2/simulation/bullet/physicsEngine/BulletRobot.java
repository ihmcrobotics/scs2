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

public class BulletRobot extends RobotExtension 
{
   private final BulletRobotPhysics robotPhysics;
   private final RigidBodyDefinition rootBodyDefinition;
   private final SimRigidBodyBasics rootSimRigidBodyBasics;
   private final SimFloatingRootJoint rootSimFloatingRootJoint;
   private final BulletRobotLinkRoot rootLink;
   private final HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<>();
   private final ArrayList<BulletRobotLinkBasics> allLinks = new ArrayList<>();
   private final ArrayList<BulletRobotLinkRevolute> afterRootLinks = new ArrayList<>();
   private final YoRegistry yoRegistry;

   public BulletRobot(Robot robot, YoRegistry physicsRegistry, BulletPhysicsEngine bulletPhysicsEngine)
   {
      super(robot, physicsRegistry);
      robotPhysics = new BulletRobotPhysics(this);

      yoRegistry = new YoRegistry(getRobotDefinition().getName() + getClass().getSimpleName());
      robot.getRegistry().addChild(yoRegistry);

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
      rootLink = new BulletRobotLinkRoot(rootSixDoFJointDefinition, rootSimFloatingRootJoint, jointNameToBulletJointIndexMap, yoRegistry);
      initializeLinkLists(rootLink, true);

      rootLink.setup(bulletPhysicsEngine);

      for (BulletRobotLinkRevolute link : afterRootLinks)
      {
         link.setBulletMultiBody(rootLink.getBulletMultiBody());
         link.setup(bulletPhysicsEngine);
      }
      rootLink.getBulletMultiBody().finalizeMultiDof();
   }
   
   private void initializeLinkLists(BulletRobotLinkBasics link, boolean isRootLink)
   {
      allLinks.add(link);
      if (!isRootLink)
      {
         afterRootLinks.add((BulletRobotLinkRevolute) link);
      }

      for (BulletRobotLinkBasics child : link.getChildren())
      {
         initializeLinkLists(child, false);
      }
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

   private void copyDataFromSCSToBullet(BulletRobotLinkBasics link)
   {
      link.copyDataFromSCSToBullet();

      for (BulletRobotLinkBasics child : link.getChildren())
      {
         copyDataFromSCSToBullet(child);
      }
   }
   
   public void updateFromBulletData(BulletPhysicsEngine bulletPhysicsEngine)
   {
      rootLink.copyBulletJointDataToSCS();

      for (BulletRobotLinkRevolute afterRootLink : afterRootLinks)
      {
         afterRootLink.copyBulletJointDataToSCS();
      }
   }

   public BulletRobotLinkRoot getRootLink()
   {
      return rootLink;
   }

   public btMultiBody getBulletMultiBody()
   {
      return rootLink.getBulletMultiBody();
   }

   public ArrayList<BulletRobotLinkRevolute> getAfterRootLinks()
   {
      return afterRootLinks;
   }

   public ArrayList<BulletRobotLinkBasics> getAllLinks()
   {
      return allLinks;
   }

   public void updateSensors()
   {
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
//         joint.getAuxialiryData().update(robotPhysics.getPhysicsOutput());
      }
   }
}
