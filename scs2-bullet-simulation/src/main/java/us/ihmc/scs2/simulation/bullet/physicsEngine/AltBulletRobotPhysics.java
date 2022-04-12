package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;

// FIXME: Not sure if we need this class.
public class AltBulletRobotPhysics
{
   private final RobotInterface owner;
   private final ReferenceFrame inertialFrame;

   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry = new RigidBodyWrenchRegistry();
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RobotPhysicsOutput physicsOutput;

   public AltBulletRobotPhysics(RobotInterface owner)
   {
      this.owner = owner;
      inertialFrame = owner.getInertialFrame();

      SimRigidBodyBasics rootBody = owner.getRootBody();

      spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, inertialFrame, false);
      spatialAccelerationCalculator.setGravitionalAcceleration(-9.81); // FIXME This should be set wherever we set it for Bullet
      physicsOutput = new RobotPhysicsOutput(spatialAccelerationCalculator, null, rigidBodyWrenchRegistry, null);
   }

   public void reset()
   {
      rigidBodyWrenchRegistry.reset();
   }

   public void update()
   {
      spatialAccelerationCalculator.reset();
   }

   public RobotPhysicsOutput getPhysicsOutput()
   {
      return physicsOutput;
   }

   public RigidBodyWrenchRegistry getRigidBodyWrenchRegistry()
   {
      return rigidBodyWrenchRegistry;
   }

}
