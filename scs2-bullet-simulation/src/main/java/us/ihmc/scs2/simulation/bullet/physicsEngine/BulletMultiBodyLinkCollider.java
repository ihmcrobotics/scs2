package us.ihmc.scs2.simulation.bullet.physicsEngine;

import com.badlogic.gdx.physics.bullet.collision.btCollisionShape;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyLinkCollider;

public class BulletMultiBodyLinkCollider 
{
	btMultiBodyLinkCollider multiBodyLinkCollider;
	int collisionGroup;
	int collisionGroupMask;
	
	public BulletMultiBodyLinkCollider (btMultiBody multibody, int index)
	{
		createBulletMultiBodyLinkCollider(multibody, index, 2, 1 + 2);
	}

	public void createBulletMultiBodyLinkCollider (btMultiBody multibody, int index, int collisionGroup, int collisionGroupMask)
	{
		multiBodyLinkCollider = new btMultiBodyLinkCollider(multibody, index);
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
	
	public void setFriction(float frict)
	{
		multiBodyLinkCollider.setFriction(frict);
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
}
