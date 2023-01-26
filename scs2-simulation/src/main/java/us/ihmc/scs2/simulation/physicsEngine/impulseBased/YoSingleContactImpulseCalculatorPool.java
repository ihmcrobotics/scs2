package us.ihmc.scs2.simulation.physicsEngine.impulseBased;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSingleContactImpulseCalculatorPool
{
   private RecyclingArrayList<YoSingleContactImpulseCalculator> pool;

   public YoSingleContactImpulseCalculatorPool(int initialCapacity,
                                               String prefix,
                                               ReferenceFrame rootFrame,
                                               RigidBodyBasics rootBodyA,
                                               ForwardDynamicsCalculator forwardDynamicsCalculatorA,
                                               RigidBodyBasics rootBodyB,
                                               ForwardDynamicsCalculator forwardDynamicsCalculatorB,
                                               YoRegistry registry)
   {
      pool = new RecyclingArrayList<>(initialCapacity, SupplierBuilder.indexedSupplier(identifier ->
      {
         return new YoSingleContactImpulseCalculator(prefix,
                                                     identifier,
                                                     rootFrame,
                                                     rootBodyA,
                                                     forwardDynamicsCalculatorA,
                                                     rootBodyB,
                                                     forwardDynamicsCalculatorB,
                                                     registry);
      }));
      clear();
   }

   public YoSingleContactImpulseCalculator nextAvailable()
   {
      return pool.add();
   }

   public void clear()
   {
      for (int i = 0; i < pool.size(); i++)
      {
         pool.get(i).clear();
      }
      pool.clear();
   }
}
