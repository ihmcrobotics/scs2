package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.RobotPhysicsOutput;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.screwTools.RigidBodyWrenchRegistry;
import us.ihmc.scs2.simulation.screwTools.SingleRobotFirstOrderIntegrator;

public class BulletBasedRobotPhysics 
{
   private final RobotInterface owner;
   private final ReferenceFrame inertialFrame;
   private final List<Collidable> collidables;
   
   private final RigidBodyWrenchRegistry rigidBodyWrenchRegistry = new RigidBodyWrenchRegistry();
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;
   private final SingleRobotFirstOrderIntegrator integrator;
   private final RobotPhysicsOutput physicsOutput;
   
   public BulletBasedRobotPhysics(RobotInterface owner)
   {
      this.owner = owner;
      inertialFrame = owner.getInertialFrame();
      
      SimRigidBodyBasics rootBody = owner.getRootBody();
      collidables = rootBody.subtreeStream().flatMap(body -> body.getCollidables().stream()).collect(Collectors.toList());
      
      integrator = new SingleRobotFirstOrderIntegrator();
      
      forwardDynamicsCalculator = new ForwardDynamicsCalculator(owner);
      physicsOutput = new RobotPhysicsOutput(forwardDynamicsCalculator.getAccelerationProvider(), null, rigidBodyWrenchRegistry, null);
   }
   
   public void updateCollidableBoundingBoxes()
   {
      collidables.forEach(collidable -> collidable.updateBoundingBox(inertialFrame));
   }
   
   public void integrateState(double dt)
   {
      physicsOutput.setDT(dt);
      integrator.integrate(dt, owner);
   }
   
   public List<Collidable> getCollidables()
   {
      return collidables;
   }
   
   public RobotPhysicsOutput getPhysicsOutput()
   {
      return physicsOutput;
   }
   
}
