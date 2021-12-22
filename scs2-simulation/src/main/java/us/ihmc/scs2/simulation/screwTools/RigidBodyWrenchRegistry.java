package us.ihmc.scs2.simulation.screwTools;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public class RigidBodyWrenchRegistry implements Function<RigidBodyReadOnly, WrenchReadOnly>
{
   private final Map<RigidBodyReadOnly, Wrench> rigidBodyWrenchMap = new HashMap<>();
   private final Function<RigidBodyReadOnly, Wrench> wrenchFactory = body -> new Wrench(body.getBodyFixedFrame(), body.getBodyFixedFrame());

   public RigidBodyWrenchRegistry()
   {
   }

   public void reset()
   {
      rigidBodyWrenchMap.clear();
   }

   public void addWrench(RigidBodyReadOnly target, WrenchReadOnly wrenchToAdd)
   {
      Wrench wrench = rigidBodyWrenchMap.computeIfAbsent(target, wrenchFactory);
      wrench.setMatchingFrame(wrenchToAdd);
   }

   @Override
   public WrenchReadOnly apply(RigidBodyReadOnly query)
   {
      return rigidBodyWrenchMap.get(query);
   }
}
