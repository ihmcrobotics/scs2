package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;

public class BulletMultiBodyRobot 
{
	private btMultiBody bulletMultiBodyRobot;
	
    public BulletMultiBodyRobot(int numberOfLinks,
    		float rootBodyMass,
    		Vector3 rootBodyIntertia,
    		YoBulletMultiBodyParameters bulletMultiBodyParameters) 
    {
		bulletMultiBodyRobot = new btMultiBody(numberOfLinks, rootBodyMass, rootBodyIntertia, bulletMultiBodyParameters.getFixedBase(), bulletMultiBodyParameters.getCanSleep());
		
		bulletMultiBodyRobot.setHasSelfCollision(bulletMultiBodyParameters.getHasSelfCollision());
		bulletMultiBodyRobot.setLinearDamping(bulletMultiBodyParameters.getLinearDamping());
		bulletMultiBodyRobot.setAngularDamping(bulletMultiBodyParameters.getAngularDamping());
	}

	public btMultiBody getBulletMultiBodyRobot() {
		return bulletMultiBodyRobot;
	}
}
