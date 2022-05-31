package us.ihmc.scs2.simulation;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulationEnergyStatistics
{
   public static void setupSimulationEnergyStatistics(SimulationSession simulationSession)
   {
      YoRegistry registry = new YoRegistry(SimulationEnergyStatistics.class.getSimpleName());
      simulationSession.getPhysicsEngine().getPhysicsEngineRegistry().addChild(registry);
     
      for (Robot robot : simulationSession.getPhysicsEngine().getRobots())
      {
         simulationSession.addAfterPhysicsCallback(new RobotEnergyStatistics(robot, simulationSession.getGravity(), registry));
      }
   }

   private static class RobotEnergyStatistics implements TimeConsumer
   {
      private final YoDouble kineticEnergy;
      private final YoDouble potentialEnergy;
      private final YoDouble orbitalEnergy;
      private final Vector3DReadOnly gravity;
      
      private List<? extends RigidBodyReadOnly> allRigidBodies;

      public RobotEnergyStatistics(Robot robot, Vector3DReadOnly gravity, YoRegistry registry)
      {
         this.gravity = gravity;
         kineticEnergy = new YoDouble(robot.getName() + "KineticEnergy", registry);
         potentialEnergy = new YoDouble(robot.getName() + "PotentialEnergy", registry);
         orbitalEnergy = new YoDouble(robot.getName() + "OrbitalEnergy", registry);
         allRigidBodies = robot.getRootBody().subtreeStream().filter(body -> !body.isRootBody()).collect(Collectors.toList());
      }

      @Override
      public void accept(double time)
      {
         double kinetic = 0.0;
         double potential = 0.0;

         for (RigidBodyReadOnly rigidBody : allRigidBodies)
         {
            SpatialInertiaReadOnly inertia = rigidBody.getInertia();
            MovingReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();

            kinetic += inertia.computeKineticCoEnergy(bodyFixedFrame.getTwistOfFrame());
            potential += -inertia.getMass() * gravity.dot(bodyFixedFrame.getTransformToRoot().getTranslation());
         }
         
         kineticEnergy.set(kinetic);
         potentialEnergy.set(potential);
         orbitalEnergy.set(kinetic + potential);
      }
   }
}
