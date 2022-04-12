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

public class AltBulletRobot extends RobotExtension 
{
   private final AltBulletRobotPhysics robotPhysics;
   private final RigidBodyDefinition rootBodyDefinition;
   private final SimRigidBodyBasics rootSimRigidBodyBasics;
   private final SimFloatingRootJoint rootSimFloatingRootJoint;
   private final AltBulletRobotLinkRoot rootLink;
   private final HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<>();
   private final ArrayList<AltBulletRobotLinkBasics> allLinks = new ArrayList<>();
   private final ArrayList<AltBulletRobotLinkRevolute> afterRootLinks = new ArrayList<>();
   private final YoRegistry yoRegistry;

   public AltBulletRobot(Robot robot, YoRegistry physicsRegistry, AltBulletPhysicsEngine bulletPhysicsEngine)
   {
      super(robot, physicsRegistry);
      robotPhysics = new AltBulletRobotPhysics(this);

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
      rootLink = new AltBulletRobotLinkRoot(rootSixDoFJointDefinition,
                                         rootSimFloatingRootJoint,
                                         jointNameToBulletJointIndexMap,
                                         robotPhysics.getRigidBodyWrenchRegistry(),
                                         yoRegistry);
      initializeLinkLists(rootLink, true);

      rootLink.setup(bulletPhysicsEngine);

      for (AltBulletRobotLinkRevolute link : afterRootLinks)
      {
         link.setBulletMultiBody(rootLink.getBulletMultiBody());
         link.setup(bulletPhysicsEngine);
      }
      rootLink.getBulletMultiBody().finalizeMultiDof();
   }
   
   private void initializeLinkLists(AltBulletRobotLinkBasics link, boolean isRootLink)
   {
      allLinks.add(link);
      if (!isRootLink)
      {
         afterRootLinks.add((AltBulletRobotLinkRevolute) link);
      }

      for (AltBulletRobotLinkBasics child : link.getChildren())
      {
         initializeLinkLists(child, false);
      }
   }

   @Override
   public void saveRobotBeforePhysicsState()
   {
      super.saveRobotBeforePhysicsState();
      
      copyDataFromSCSToBullet();
   }

   public void copyDataFromSCSToBullet()
   {
      robotPhysics.reset();
      copyDataFromSCSToBullet(rootLink);
   }

   private void copyDataFromSCSToBullet(AltBulletRobotLinkBasics link)
   {
      link.copyDataFromSCSToBullet();

      for (AltBulletRobotLinkBasics child : link.getChildren())
      {
         copyDataFromSCSToBullet(child);
      }
   }
   
   public void updateFromBulletData(AltBulletPhysicsEngine bulletPhysicsEngine, double dt)
   {
      rootLink.copyBulletJointDataToSCS(dt);

      for (AltBulletRobotLinkRevolute afterRootLink : afterRootLinks)
      {
         afterRootLink.copyBulletJointDataToSCS(dt);
      }
      robotPhysics.update();
   }

   public AltBulletRobotLinkRoot getRootLink()
   {
      return rootLink;
   }

   public btMultiBody getBulletMultiBody()
   {
      return rootLink.getBulletMultiBody();
   }

   public ArrayList<AltBulletRobotLinkRevolute> getAfterRootLinks()
   {
      return afterRootLinks;
   }

   public ArrayList<AltBulletRobotLinkBasics> getAllLinks()
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
