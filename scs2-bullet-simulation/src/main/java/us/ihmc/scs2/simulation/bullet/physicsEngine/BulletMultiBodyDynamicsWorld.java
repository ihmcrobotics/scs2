package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.ArrayList;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.btBroadphaseInterface;
import com.badlogic.gdx.physics.bullet.collision.btCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.collision.btCollisionDispatcher;
import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
import com.badlogic.gdx.physics.bullet.collision.btDbvtBroadphase;
import com.badlogic.gdx.physics.bullet.collision.btDefaultCollisionConfiguration;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBody;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraint;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyConstraintSolver;
import com.badlogic.gdx.physics.bullet.dynamics.btMultiBodyDynamicsWorld;
import com.badlogic.gdx.physics.bullet.dynamics.btRigidBody;

import us.ihmc.scs2.definition.robot.JointDefinition;

public class BulletMultiBodyDynamicsWorld {
	private btCollisionConfiguration collisionConfiguration;
	private btCollisionDispatcher collisionDispatcher;
	private btBroadphaseInterface broadphaseInterface;
	private btMultiBodyConstraintSolver solver;
	private btMultiBodyDynamicsWorld multiBodyDynamicsWorld;
	private final ArrayList<btRigidBody> rigidBodies = new ArrayList<>();
	private final ArrayList<BulletMultiBodyRobot> multiBodies = new ArrayList<>();
	private final ArrayList<btCollisionObject> collisionObjects = new ArrayList<>();
	// private final ArrayList<BulletTerrainObject> terrainObjects = new
	// ArrayList<>();

	public BulletMultiBodyDynamicsWorld() {
		collisionConfiguration = new btDefaultCollisionConfiguration();
		collisionDispatcher = new btCollisionDispatcher(collisionConfiguration);
		broadphaseInterface = new btDbvtBroadphase();
		solver = new btMultiBodyConstraintSolver();
		multiBodyDynamicsWorld = new btMultiBodyDynamicsWorld(collisionDispatcher, broadphaseInterface, solver,
				collisionConfiguration);
		Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
		multiBodyDynamicsWorld.setGravity(gravity);
	}

	public int stepSimulation(double dt) {
		int maxSubSteps = 1; // With SCS, we want every tick to get a buffer entry and step through things
							 // one step at a time.
		float fixedTimeStep = (float) dt; // SCS has a fixed timestep already so let's just use it
		float timePassedSinceThisWasCalledLast = fixedTimeStep; // We are essentially disabling interpolation here

		return getMultiBodyDynamicsWorld().stepSimulation(timePassedSinceThisWasCalledLast, maxSubSteps, fixedTimeStep);
	}

	public btMultiBodyDynamicsWorld getMultiBodyDynamicsWorld() {
		return multiBodyDynamicsWorld;
	}

	public void dispose() {
		System.out.println("Destroy");
//		for (btRigidBody rigidBody : rigidBodies) {
//			multiBodyDynamicsWorld.removeRigidBody(rigidBody);
//			rigidBody.dispose();
//		}
//
//		for (BulletMultiBodyRobot multiBody : multiBodies) {
//			multiBodyDynamicsWorld.removeMultiBody(multiBody.getBulletMultiBody());
//			multiBody.getBulletMultiBody().dispose();
//		}
//
//		for (btCollisionObject shape : collisionObjects) {
//			multiBodyDynamicsWorld.removeCollisionObject(shape);
//			shape.dispose();
//		}
//		collisionObjects.clear();
//
//		multiBodyDynamicsWorld.dispose();
//		collisionConfiguration.dispose();
//		collisionDispatcher.dispose();
//		broadphaseInterface.dispose();
//		solver.dispose();
	}

	public void addMultiBody(BulletMultiBodyRobot bulletMultiBody) 
	{
		multiBodies.add(bulletMultiBody);
        for (BulletMultiBodyLinkCollider linkCollider : bulletMultiBody.getBulletMultiBodyLinkColliderArray())
        {
        	multiBodyDynamicsWorld.addCollisionObject(linkCollider.getMultiBodyLinkCollider(), linkCollider.getCollisionGroup(),
        			linkCollider.getCollisionGroupMask());
        }
        for (btMultiBodyConstraint constraint : bulletMultiBody.getBulletMultiBodyConstrantArray())
        {
        	multiBodyDynamicsWorld.addMultiBodyConstraint(constraint);
        }
        bulletMultiBody.finalizeMultiDof();
		multiBodyDynamicsWorld.addMultiBody(bulletMultiBody.getBulletMultiBody());
	}
}
