package us.ihmc.scs2.simulation.screwTools;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialImpulse;
import us.ihmc.mecano.spatial.interfaces.SpatialImpulseReadOnly;

public class RigidBodyImpulseRegistry implements Function<RigidBodyReadOnly, SpatialImpulseReadOnly>
{
   private final Map<RigidBodyReadOnly, SpatialImpulse> rigidBodyImpulseMap = new HashMap<>();
   private final Function<RigidBodyReadOnly, SpatialImpulse> impulseFactory = body -> new SpatialImpulse(body.getBodyFixedFrame(), body.getBodyFixedFrame());

   public RigidBodyImpulseRegistry()
   {
   }

   public void reset()
   {
      rigidBodyImpulseMap.clear();
   }

   public void addImpulse(RigidBodyReadOnly target, SpatialImpulseReadOnly impulseToAdd)
   {
      SpatialImpulse impulse = rigidBodyImpulseMap.computeIfAbsent(target, impulseFactory);
      impulse.setMatchingFrame(impulseToAdd);
   }

   @Override
   public SpatialImpulseReadOnly apply(RigidBodyReadOnly query)
   {
      return rigidBodyImpulseMap.get(query);
   }
}
