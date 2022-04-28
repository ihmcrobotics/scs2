package us.ihmc.scs2.simulation.screwTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public class RigidBodyWrenchRegistry implements Function<RigidBodyReadOnly, WrenchReadOnly>
{
   private final Map<RigidBodyReadOnly, Wrench> rigidBodyWrenchMap = new HashMap<>();
   private final List<Wrench> wrenchCache = new ArrayList<>();
   private final Function<RigidBodyReadOnly, Wrench> wrenchFactory = body -> {
      
      Wrench wrench = new Wrench(body.getBodyFixedFrame(), body.getBodyFixedFrame());
      wrenchCache.add(wrench);
      return wrench;
   };

   public RigidBodyWrenchRegistry()
   {
   }

   public void reset()
   {
      for (int i = 0; i < wrenchCache.size(); i++)
      {
         wrenchCache.get(i).setToZero();
      }
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
