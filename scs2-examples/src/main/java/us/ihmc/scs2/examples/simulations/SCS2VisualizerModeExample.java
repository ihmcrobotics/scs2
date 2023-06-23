package us.ihmc.scs2.examples.simulations;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.yoVariables.variable.YoInteger;

public class SCS2VisualizerModeExample
{
   public static void main(String[] args)
   {
      SimulationConstructionSet2 simulation = new SimulationConstructionSet2(SCS2VisualizerModeExample.class.getSimpleName(),
                                                                             PhysicsEngineFactory.newDoNothingPhysicsEngineFactory());
      simulation.getSimulationSession().getSimulationSessionControls().setBufferRecordTickPeriod(1);
      YoInteger index = new YoInteger("index", simulation.getRootRegistry());

      boolean waitUntilVisualizerFullyUp = true;
      boolean stopSimulationThread = true;
      boolean disableJavaFXImplicitExit = false;
      simulation.start(waitUntilVisualizerFullyUp, stopSimulationThread, disableJavaFXImplicitExit);

      for (int i = 0; i < 100; i++)
      {
         index.set(i);
         simulation.simulateNow(1);
      }

      simulation.startSimulationThread();
      // This example highlights the issue #117: When the GUI shows, the charts won't necessarily be complete, it is quite unreliable.
   }
}
