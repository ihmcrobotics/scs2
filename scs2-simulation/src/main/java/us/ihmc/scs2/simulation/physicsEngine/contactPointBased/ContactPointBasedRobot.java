package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHolder;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class ContactPointBasedRobot extends RobotExtension implements CollidableHolder
{
   private final ContactPointBasedRobotPhysics robotPhysics;

   public ContactPointBasedRobot(Robot robot, YoRegistry physicsRegistry)
   {
      super(robot, physicsRegistry);
      robotPhysics = new ContactPointBasedRobotPhysics(this);
   }

   public void enableJointWrenchCalculator()
   {
      robotPhysics.enableJointWrenchCalculator();
   }

   public void resetCalculators()
   {
      robotPhysics.resetCalculators();
   }

   public void computeJointLowLevelControl()
   {
      robotPhysics.computeJointLowLevelControl();
   }

   public void addRigidBodyExternalWrench(RigidBodyReadOnly target, WrenchReadOnly wrenchToAdd)
   {
      robotPhysics.addRigidBodyExternalWrench(target, wrenchToAdd);
   }

   public void doForwardDynamics(Vector3DReadOnly gravity)
   {
      robotPhysics.doForwardDynamics(gravity);
   }

   public void computeJointWrenches(double dt)
   {
      robotPhysics.computeJointWrenches(dt);
   }

   public void updateCollidableBoundingBoxes()
   {
      robotPhysics.updateCollidableBoundingBoxes();
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculator()
   {
      return robotPhysics.getForwardDynamicsCalculator();
   }

   public void writeJointAccelerations()
   {
      robotPhysics.writeJointAccelerations();
   }

   public void integrateState(double dt)
   {
      robotPhysics.integrateState(dt);
   }

   public void updateSensors()
   {
      for (SimJointBasics joint : getJointsToConsider())
      {
         joint.getAuxiliaryData().update(robotPhysics.getPhysicsOutput());
      }
   }

   @Override
   public List<Collidable> getCollidables()
   {
      return robotPhysics.getCollidables();
   }
}
