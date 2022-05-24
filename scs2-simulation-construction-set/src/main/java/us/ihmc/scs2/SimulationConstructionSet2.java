package us.ihmc.scs2;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.TimeConsumer;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoVariable;

public class SimulationConstructionSet2 implements YoVariableHolder
{
   public static final ReferenceFrame inertialFrame = SimulationSession.DEFAULT_INERTIAL_FRAME;

   private SimulationSession simulationSession;
   private SessionVisualizerControls visualizerControls;

   private boolean guiEnabled;

   public static PhysicsEngineFactory contactPointBasedPhysicsEngineFactory()
   {
      return contactPointBasedPhysicsEngineFactory(null);
   }

   public static PhysicsEngineFactory contactPointBasedPhysicsEngineFactory(ContactPointBasedContactParameters contactPointBasedContactParameters)
   {
      return (inertialFrame, rootRegistry) ->
      {
         ContactPointBasedPhysicsEngine physicsEngine = new ContactPointBasedPhysicsEngine(inertialFrame, rootRegistry);
         if (contactPointBasedContactParameters != null)
            physicsEngine.setGroundContactParameters(contactPointBasedContactParameters);
         return physicsEngine;
      };
   }

   public static PhysicsEngineFactory impulseBasedPhysicsEngineFactory()
   {
      return impulseBasedPhysicsEngineFactory(null);
   }

   public static PhysicsEngineFactory impulseBasedPhysicsEngineFactory(ContactParametersReadOnly contactParameters)
   {
      return (inertialFrame, rootRegistry) ->
      {
         ImpulseBasedPhysicsEngine physicsEngine = new ImpulseBasedPhysicsEngine(inertialFrame, rootRegistry);
         if (contactParameters != null)
            physicsEngine.setGlobalContactParameters(contactParameters);
         return physicsEngine;
      };
   }

   public SimulationConstructionSet2(String simulationName)
   {
      this(simulationName, contactPointBasedPhysicsEngineFactory());
   }

   public SimulationConstructionSet2(String simulationName, PhysicsEngineFactory physicsEngineFactory)
   {
      simulationSession = new SimulationSession(inertialFrame, simulationName, physicsEngineFactory);
   }

   public YoBufferPropertiesReadOnly getBufferProperties()
   {
      return simulationSession.getBufferProperties();
   }

   public PhysicsEngine getPhysicsEngine()
   {
      return simulationSession.getPhysicsEngine();
   }

   public SimulationSessionControls getSimulationControls()
   {
      return simulationSession.getSimulationSessionControls();
   }

   public void setGUIEnabled(boolean guiEnabled)
   {
      this.guiEnabled = guiEnabled;
   }

   public boolean isGUIEnabled()
   {
      return guiEnabled;
   }

   public double getDT()
   {
      return simulationSession.getSessionDTSeconds();
   }

   public void setDT(double dt)
   {
      simulationSession.setSessionDTSeconds(dt);
   }

   public void setRecordFrequency(int recordFrequency)
   {
      simulationSession.setBufferRecordTickPeriod(recordFrequency);
   }

   public void setRecordDT(double recordDT)
   {
      simulationSession.setBufferRecordTickPeriod((int) (recordDT / getDT()));
   }

   public double getRecordDT()
   {
      return simulationSession.getBufferRecordTimePeriod();
   }

   public int getRecordFrequency()
   {
      return simulationSession.getBufferRecordTickPeriod();
   }

   public int getCurrentIndex()
   {
      return getBufferProperties().getCurrentIndex();
   }

   public List<? extends Robot> getRobots()
   {
      return getPhysicsEngine().getRobots();
   }

   public void addBeforePhysicsCallback(TimeConsumer beforePhysicsCallback)
   {
      simulationSession.addBeforePhysicsCallback(beforePhysicsCallback);
   }

   public void addAfterPhysicsCallback(TimeConsumer afterPhysicsCallback)
   {
      simulationSession.addAfterPhysicsCallback(afterPhysicsCallback);
   }

   public YoRegistry getRootRegistry()
   {
      return simulationSession.getRootRegistry();
   }

   public void addRegistry(YoRegistry registry)
   {
      getRootRegistry().addChild(registry);
   }

   @Override
   public List<YoVariable> getVariables()
   {
      return getRootRegistry().getVariables();
   }

   @Override
   public YoVariable findVariable(String namespaceEnding, String name)
   {
      return getRootRegistry().findVariable(namespaceEnding, name);
   }

   @Override
   public List<YoVariable> findVariables(String namespaceEnding, String name)
   {
      return getRootRegistry().findVariables(namespaceEnding, name);
   }

   @Override
   public List<YoVariable> findVariables(YoNamespace namespace)
   {
      return getRootRegistry().findVariables(namespace);
   }

   @Override
   public boolean hasUniqueVariable(String namespaceEnding, String name)
   {
      return getRootRegistry().hasUniqueVariable(namespaceEnding, name);
   }

   public void setPlaybackRealTimeRate(double realTimeRate)
   {
      simulationSession.submitPlaybackRealTimeRate(realTimeRate);
   }

   public double getPlaybackRealTimeRate()
   {
      return simulationSession.getPlaybackRealTimeRate();
   }

   public void addSaticVisual(VisualDefinition visualDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addStaticVisual(visualDefinition);
   }

   public void addSaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      if (visualizerControls != null)
         visualizerControls.addStaticVisuals(visualDefinitions);
   }

   public void removeStaticVisual(VisualDefinition visualDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.removeStaticVisual(visualDefinition);
   }

   public void removeStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      if (visualizerControls != null)
         visualizerControls.removeStaticVisuals(visualDefinitions);
   }

   public void setCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      if (visualizerControls != null)
         visualizerControls.requestCameraRigidBodyTracking(robotName, rigidBodyName);
   }

   public void setCameraOrientation(double latitude, double longitude, double roll)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraOrientation(latitude, longitude, roll);
   }

   public void setCameraPosition(double x, double y, double z)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraPosition(x, y, z);
   }

   public void setCameraFocusPosition(double x, double y, double z)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraFocusPosition(x, y, z);
   }

   public void setCameraZoom(double distanceFromFocus)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraZoom(distanceFromFocus);
   }

   public void addShutdownListener(Runnable listener)
   {
      simulationSession.addShutdownListener(listener);
   }

   public void start()
   {
      simulationSession.startSessionThread();

      if (guiEnabled)
      {
         visualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
         visualizerControls.addVisualizerShutdownListener(this::destroy);
      }
   }

   private boolean hasBeenDestroyed;

   public void destroy()
   {
      if (hasBeenDestroyed)
         return;

      LogTools.info("Destroying simulation");
      hasBeenDestroyed = true;

      // TODO Destroy stuff!
   }

   public void tickAndUpdate()
   {
      simulationSession.stopSessionThread();
      simulationSession.submitIncrementBufferIndexRequestAndWait(1);
      simulationSession.getBuffer().writeBuffer();
      simulationSession.startSessionThread();
   }

   public void updateAndTick()
   {
      simulationSession.stopSessionThread();
      simulationSession.getBuffer().writeBuffer();
      simulationSession.submitIncrementBufferIndexRequestAndWait(1);
      simulationSession.startSessionThread();
   }

   public void tick()
   {
      simulationSession.submitIncrementBufferIndexRequest(1);
   }

   public void setupEntryBox(String varName)
   {
      visualizerControls.addYoEntry(varName);
   }

   public void setupEntryBox(String[] varNames)
   {
      visualizerControls.addYoEntry(Arrays.asList(varNames));
   }
}
