package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;

public class BulletMultiBodyLinkCollider 
{
	btMultiBodyLinkCollider multiBodyLinkCollider;
	String jointName;
	int linkColliderIndex;
	int collisionGroup;
	int collisionGroupMask;
	
	public BulletMultiBodyLinkCollider (btMultiBody multibody, int index, String jointName)
	{
		createBulletMultiBodyLinkCollider(multibody, index, jointName, 2, 1 + 2);
	}

	public void createBulletMultiBodyLinkCollider (btMultiBody multibody, int index, String jointName, int collisionGroup, int collisionGroupMask)
	{
		multiBodyLinkCollider = new btMultiBodyLinkCollider(multibody, index);
		this.linkColliderIndex = index;
		this.jointName = jointName;
		this.collisionGroup = collisionGroup;
		this.collisionGroupMask = collisionGroupMask;
	}
	
	public void setCollisionGroupMask (int collisionGroup, int collisionGroupMask)
	{
		this.collisionGroup = collisionGroup;
		this.collisionGroupMask = collisionGroupMask;
	}
	
	public void setCollisionShape(btCollisionShape shape)
	{
		multiBodyLinkCollider.setCollisionShape(shape);
	}
	
	public void setFriction(double friction)
	{
		multiBodyLinkCollider.setFriction((float)friction);
	}
	
	public void setRestitution (double restitution)
	{
	   multiBodyLinkCollider.setRestitution((float)restitution);
	}
	
   public void setHitFraction(double hitFraction)
   {
      multiBodyLinkCollider.setHitFraction((float) hitFraction);
   }
   
   public void setRollingFriction(double rollingFriction)
   {
      multiBodyLinkCollider.setRollingFriction((float) rollingFriction);
   }
   
   public void setSpinningFriction(double spinningFriction)
   {
      multiBodyLinkCollider.setSpinningFriction((float) spinningFriction);
   }
   
   public void setContactProcessingThreshold(double contactProcessingThreshold)
   {
      multiBodyLinkCollider.setContactProcessingThreshold((float) contactProcessingThreshold);
   }
	
	public btMultiBodyLinkCollider getMultiBodyLinkCollider()
	{
		return multiBodyLinkCollider;
	}
	
	public int getCollisionGroup()
	{
		return collisionGroup;
	}
	public int getCollisionGroupMask()
	{
		return collisionGroupMask;
	}
	
	public String getJointName()
	{
	   return jointName;
	}
	public int getLinkColliderIndex()
	{
	   return linkColliderIndex;
	}
}
