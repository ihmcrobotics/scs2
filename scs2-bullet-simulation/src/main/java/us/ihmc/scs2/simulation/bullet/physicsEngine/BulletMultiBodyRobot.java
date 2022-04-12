package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;

public class BulletMultiBodyRobot 
{
	private btMultiBody bulletMultiBodyRobot;
	
    public BulletMultiBodyRobot(int numberOfLinks,
    		float rootBodyMass,
    		Vector3 rootBodyIntertia,
    		BulletContactParameters bulletContactParameters) 
    {
		bulletMultiBodyRobot = new btMultiBody(numberOfLinks, rootBodyMass, rootBodyIntertia, bulletContactParameters.getFixedBase(), bulletContactParameters.getCanSleep());
		
		bulletMultiBodyRobot.setHasSelfCollision(bulletContactParameters.getHasSelfCollision());
		bulletMultiBodyRobot.setLinearDamping(bulletContactParameters.getLinearDamping());
		bulletMultiBodyRobot.setAngularDamping(bulletContactParameters.getAngularDamping());
	}

	public btMultiBody getBulletMultiBodyRobot() {
		return bulletMultiBodyRobot;
	}
}
