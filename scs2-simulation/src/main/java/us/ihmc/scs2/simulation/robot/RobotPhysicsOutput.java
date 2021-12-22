package us.ihmc.scs2.simulation.robot;

import java.util.function.Function;

import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public class RobotPhysicsOutput
{
   private final RigidBodyAccelerationProvider accelerationProvider;
   private final RigidBodyTwistProvider deltaTwistProvider;
   private final Function<RigidBodyReadOnly, WrenchReadOnly> externalWrenchProvider;
   private final Function<RigidBodyReadOnly, SpatialImpulseReadOnly> externalImpulseProvider;

   private double dt;

   public RobotPhysicsOutput(RigidBodyAccelerationProvider accelerationProvider, RigidBodyTwistProvider deltaTwistProvider,
                             Function<RigidBodyReadOnly, WrenchReadOnly> externalWrenchProvider,
                             Function<RigidBodyReadOnly, SpatialImpulseReadOnly> externalImpulseProvider)
   {
      super();
      this.accelerationProvider = accelerationProvider;
      this.deltaTwistProvider = deltaTwistProvider;
      this.externalWrenchProvider = externalWrenchProvider;
      this.externalImpulseProvider = externalImpulseProvider;
   }

   public void setDT(double dt)
   {
      this.dt = dt;
   }

   public double getDT()
   {
      return dt;
   }

   public RigidBodyAccelerationProvider getAccelerationProvider()
   {
      return accelerationProvider;
   }

   public RigidBodyTwistProvider getDeltaTwistProvider()
   {
      return deltaTwistProvider;
   }

   public Function<RigidBodyReadOnly, WrenchReadOnly> getExternalWrenchProvider()
   {
      return externalWrenchProvider;
   }

   public Function<RigidBodyReadOnly, SpatialImpulseReadOnly> getExternalImpulseProvider()
   {
      return externalImpulseProvider;
   }
}
