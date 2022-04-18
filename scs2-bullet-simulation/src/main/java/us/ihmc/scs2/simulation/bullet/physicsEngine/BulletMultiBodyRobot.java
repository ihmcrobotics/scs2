package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;
import java.util.HashMap;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraint;

public class BulletMultiBodyRobot 
{
	private btMultiBody bulletMultiBodyRobot;
	private HashMap<String, Integer> jointNameToBulletJointIndexMap = new HashMap<String, Integer>();
	private final ArrayList<BulletMultiBodyLinkCollider> allLinks = new ArrayList<>();
	private final ArrayList<btMultiBodyConstraint> allConstraints = new ArrayList<>();
		
    public BulletMultiBodyRobot(int numberOfLinks,
    		float rootBodyMass,
    		Vector3 rootBodyIntertia,
    		HashMap<String, Integer> jointNameToBulletJointIndexMap,
    		YoBulletMultiBodyParameters bulletMultiBodyParameters) 
    {
		bulletMultiBodyRobot = new btMultiBody(numberOfLinks, rootBodyMass, rootBodyIntertia, bulletMultiBodyParameters.getFixedBase(), bulletMultiBodyParameters.getCanSleep());
		this.jointNameToBulletJointIndexMap = jointNameToBulletJointIndexMap;
		
		bulletMultiBodyRobot.setHasSelfCollision(bulletMultiBodyParameters.getHasSelfCollision());
		bulletMultiBodyRobot.setLinearDamping(bulletMultiBodyParameters.getLinearDamping());
		bulletMultiBodyRobot.setAngularDamping(bulletMultiBodyParameters.getAngularDamping());
	}

	public btMultiBody getBulletMultiBody() {
		return bulletMultiBodyRobot;
	}
	
	public ArrayList<BulletMultiBodyLinkCollider> getBulletMultiBodyLinkColliderArray()
	{
		return allLinks;
	}
	
	  public BulletMultiBodyLinkCollider getBulletMultiBodyLinkCollider (int index)
	   {
	     return allLinks.get(index);
	   }
	
	public ArrayList<btMultiBodyConstraint> getBulletMultiBodyConstrantArray ()
	{
		return allConstraints;
	}
	
	public HashMap<String, Integer> getJointNameToBulletJointIndexMap() {
		return jointNameToBulletJointIndexMap;
	}

	public void addBulletMuliBodyLinkCollider(BulletMultiBodyLinkCollider bulletMultiBodyLinkCollider) 
	{
		allLinks.add(bulletMultiBodyLinkCollider);
	}
	
	public void addMultiBodyConstraint(btMultiBodyConstraint bulletMultiBodyConstraint) 
	{
		allConstraints.add(bulletMultiBodyConstraint);
	}
	
	public void finalizeMultiDof()
	{
		bulletMultiBodyRobot.finalizeMultiDof();
	}
}
