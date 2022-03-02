package us.ihmc.scs2.simulation.bullet.physicsEngine;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.scs2.simulation.screwTools.SingleRobotFirstOrderIntegrator;

public class BulletBasedRobotPhysics 
{
   private final RobotInterface owner;
   private final ReferenceFrame inertialFrame;
  
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry = new RigidBodyWrenchRegistry();
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;
   private final SingleRobotFirstOrderIntegrator integrator;
   private final RobotPhysicsOutput physicsOutput;
   
   public BulletBasedRobotPhysics(RobotInterface owner)
   {
      this.owner = owner;
      inertialFrame = owner.getInertialFrame();
      
      SimRigidBodyBasics rootBody = owner.getRootBody();
      
      integrator = new SingleRobotFirstOrderIntegrator();
      
      forwardDynamicsCalculator = new ForwardDynamicsCalculator(owner);
      physicsOutput = new RobotPhysicsOutput(forwardDynamicsCalculator.getAccelerationProvider(), null, rigidBodyWrenchRegistry, null);
   }

   
   public void integrateState(double dt)
   {
      physicsOutput.setDT(dt);
      integrator.integrate(dt, owner);
   }
   
   
   public RobotPhysicsOutput getPhysicsOutput()
   {
      return physicsOutput;
   }
   
}
