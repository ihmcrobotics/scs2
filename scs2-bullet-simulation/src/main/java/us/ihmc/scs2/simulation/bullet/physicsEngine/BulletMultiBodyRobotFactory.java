package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.List;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btCompoundShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.simulation.robot.Robot;

public class BulletMultiBodyRobotFactory 
{
	public static BulletMultiBodyRobot newInstance(Robot robot, BulletContactParameters bulletContactParameters)
	{
		int numberOfLinks = countJoints(robot.getRootBody().getChildrenJoints().get(0)) - 1;
        float rootBodyMass = (float) robot.getRobotDefinition().getRootBodyDefinition().getMass();
	    Vector3 rootBodyIntertia = new Vector3((float) robot.getRobotDefinition().getRootBodyDefinition().getMomentOfInertia().getM00(),
	                                           (float) robot.getRobotDefinition().getRootBodyDefinition().getMomentOfInertia().getM11(),
	                                           (float) robot.getRobotDefinition().getRootBodyDefinition().getMomentOfInertia().getM22());
		
		BulletMultiBodyRobot bulletMultiBodyRobot = new BulletMultiBodyRobot(numberOfLinks, rootBodyMass, rootBodyIntertia, bulletContactParameters);
		

		return bulletMultiBodyRobot;
	}
	
	private static btMultiBodyLinkCollider createBulletCollider(BulletMultiBodyRobot bulletMultiBodyRobot, int bulletJointIndex, float friction) 
	{
		btMultiBodyLinkCollider bulletMultiBodyLinkCollider = new btMultiBodyLinkCollider(bulletMultiBodyRobot.getBulletMultiBodyRobot(), bulletJointIndex);
//		bulletMultiBodyLinkCollider.setCollisionShape(collisionSet.getBulletCompoundShape());
		bulletMultiBodyLinkCollider.setFriction(friction);

//		bulletPhysicsManager.addMultiBodyCollisionShape(bulletMultiBodyLinkCollider, collisionGroup,
//				collisionGroupMask);
		
		return bulletMultiBodyLinkCollider;
	}
	
	public static btCompoundShape createCollisionSet(List<CollisionShapeDefinition> collisionShapeDefinitions,
               ReferenceFrame frameAfterParentJoint,
               ReferenceFrame linkCenterOfMassFrame)
	{
		btCompoundShape bulletCompoundShape = new btCompoundShape();
		
		for (CollisionShapeDefinition shapeDefinition : collisionShapeDefinitions)
		{
//			AltBulletRobotLinkCollisionShape collisionShape = new AltBulletRobotLinkCollisionShape(shapeDefinition,
//			                                                                     frameAfterParentJoint,
//			                                                                     linkCenterOfMassFrame);
//			collisionShape.addToCompoundShape(bulletCompoundShape);
		}

		return bulletCompoundShape;
	}

	private static int countJoints(JointBasics joint) {
		int numberOfJoints = 1;
		for (JointBasics childrenJoint : joint.getSuccessor().getChildrenJoints()) {
			numberOfJoints += countJoints(childrenJoint);
		}
		return numberOfJoints;
	}

}
