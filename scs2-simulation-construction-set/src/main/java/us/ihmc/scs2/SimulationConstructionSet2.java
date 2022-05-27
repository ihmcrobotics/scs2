package us.ihmc.scs2;

import java.io.File;
import java.net.URL;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.Pane;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.SessionDataExportRequest;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
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
import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
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

   public Robot addRobot(RobotDefinition robotDefinition)
   {
      return simulationSession.addRobot(robotDefinition);
   }

   public void addRobot(Robot robot)
   {
      simulationSession.addRobot(robot);
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

   public void addStaticVisual(VisualDefinition visualDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addStaticVisual(visualDefinition);
   }

   public void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
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

   public void addYoGraphic(YoGraphicDefinition yoGraphicDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addYoGraphic(yoGraphicDefinition);
   }

   public void addYoGraphic(String namespace, YoGraphicDefinition yoGraphicDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addYoGraphic(namespace, yoGraphicDefinition);
   }

   public void setCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      if (visualizerControls != null)
         visualizerControls.requestCameraRigidBodyTracking(robotName, rigidBodyName);
   }

   public void setCameraOrientation(double latitude, double longitude)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraOrientation(latitude, longitude);
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

   public void startSimulationThread()
   {
      simulationSession.startSessionThread();

      if (guiEnabled)
      {
         visualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession);
         visualizerControls.addVisualizerShutdownListener(this::destroy);
      }
   }

   public void stopSimulationThread()
   {
      simulationSession.stopSessionThread();
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
      if (visualizerControls != null)
         visualizerControls.addYoEntry(varName);
   }

   public void setupEntryBox(String[] varNames)
   {
      if (visualizerControls != null)
         visualizerControls.addYoEntry(Arrays.asList(varNames));
   }

   public void setupEntryBoxGroup(String name, String[] varNames)
   {
      if (visualizerControls != null)
         visualizerControls.addYoEntry(name, Arrays.asList(varNames));
   }

   // TODO Missing setupGraph, setupGraphGroup

   public void disableGUIComponents()
   {
      if (visualizerControls != null)
         visualizerControls.disableUserControls();
   }

   public void enableGUIComponents()
   {
      if (visualizerControls != null)
         visualizerControls.enableUserControls();
   }

   /**
    * Adds a custom JavaFX control, for instance a {@link Button}, which is displayed in the user side
    * panel on the right side of the main window.
    * 
    * @param control the custom control to add.
    */
   public void addCustomControl(Node control)
   {
      if (visualizerControls != null)
         visualizerControls.addCustomControl(control);
   }

   /**
    * Removes a custom JavaFX control that was previously added via {@link #addCustomControl(Node)}.
    * 
    * @param control the control to be removed.
    * @return whether the control was found and removed successfully.
    */
   public boolean removeCustomControl(Node control)
   {
      if (visualizerControls != null)
         return visualizerControls.removeCustomControl(control);
      else
         return false;
   }

   /**
    * Loads and adds a mini-GUI from an FXML file. The GUI is displayed in the user side panel on the
    * right side of the main window.
    * 
    * @param name         the title of the new pane.
    * @param fxmlResource the locator to the FXML resource.
    */
   public void loadCustomPane(String name, URL fxmlResource)
   {
      if (visualizerControls != null)
         visualizerControls.loadCustomPane(name, fxmlResource);
   }

   /**
    * Adds a mini-GUI to the user side panel on the right side of the main window.
    * 
    * @param name the title of the new pane.
    * @param pane the pane to be added.
    */
   public void addCustomPane(String name, Pane pane)
   {
      if (visualizerControls != null)
         visualizerControls.addCustomPane(name, pane);
   }

   /**
    * Removes a pane previously added via {@link #loadCustomPane(String, URL)} or
    * {@link #addCustomPane(String, Pane)}.
    * 
    * @param name the title of the pane to remove.
    */
   public boolean removeCustomPane(String name)
   {
      if (visualizerControls != null)
         return visualizerControls.removeCustomPane(name);
      else
         return false;
   }

   public void setSimulateNoFasterThanRealTime(boolean simulateNoFasterThanRealTime)
   {
      simulationSession.submitRunAtRealTimeRate(simulateNoFasterThanRealTime);
   }

   public boolean getSimulateNoFasterThanRealTime()
   {
      return simulationSession.getRunAtRealTimeRate();
   }

   public void stop()
   {
      getSimulationControls().pause();
   }

   public boolean isSimulationThreadRunning()
   {
      return simulationSession.hasSessionStarted();
   }

   public boolean isSimulating()
   {
      return simulationSession.getActiveMode() == SessionMode.RUNNING;
   }

   public boolean isPlaying()
   {
      return simulationSession.getActiveMode() == SessionMode.PLAYBACK;
   }

   public void play()
   {
      simulationSession.setSessionMode(SessionMode.PLAYBACK);
   }

   public void simulate()
   {
      getSimulationControls().simulate();
   }

   public void simulate(double duration)
   {
      getSimulationControls().simulate(duration);
   }

   public void simulate(int numberOfTicks)
   {
      getSimulationControls().simulate(numberOfTicks);
   }

   public boolean simulateNow(double duration)
   {
      return getSimulationControls().simulateNow(duration);
   }

   public boolean simulateNow(long numberOfTicks)
   {
      return getSimulationControls().simulateNow(numberOfTicks);
   }

   public boolean simulateNow()
   {
      return getSimulationControls().simulateNow();
   }

   public void addSimulationThrowableListener(Consumer<Throwable> listener)
   {
      getSimulationControls().addSimulationThrowableListener(listener);
   }

   public void addExternalTerminalCondition(BooleanSupplier... externalTerminalConditions)
   {
      getSimulationControls().addExternalTerminalCondition(externalTerminalConditions);
   }

   public boolean removeExternalTerminalCondition(BooleanSupplier externalTerminalCondition)
   {
      return getSimulationControls().removeExternalTerminalCondition(externalTerminalCondition);
   }

   public void clearExternalTerminalConditions()
   {
      getSimulationControls().clearExternalTerminalConditions();
   }

   public void setCurrentIndex(int bufferIndexRequest)
   {
      simulationSession.submitBufferIndexRequestAndWait(bufferIndexRequest);
   }

   public void gotoInPoint()
   {
      getSimulationControls().setBufferCurrentIndexToInPoint();
   }

   public void gotoOutPoint()
   {
      getSimulationControls().setBufferCurrentIndexToOutPoint();
   }

   public int getInPoint()
   {
      return getBufferProperties().getInPoint();
   }

   public int getOutPoint()
   {
      return getBufferProperties().getOutPoint();
   }

   public void setInPoint()
   {
      getSimulationControls().setBufferInPointIndexToCurrent();
   }

   public void setOutPoint()
   {
      getSimulationControls().setBufferOutPointIndexToCurrent();
   }

   public int getBufferSize()
   {
      return getBufferProperties().getSize();
   }

   public void cropBuffer()
   {
      getSimulationControls().cropBuffer();
   }

   public void changeBufferSize(int bufferSize)
   {
      simulationSession.submitBufferSizeRequestAndWait(bufferSize);
   }

   public void exportData(SessionDataExportRequest request)
   {
      simulationSession.submitSessionDataExportRequestAndWait(request);
   }

   public void exportVideo(File file)
   {
      if (visualizerControls != null)
         visualizerControls.exportVideo(file);
   }

   public YoSharedBuffer getBuffer()
   {
      return simulationSession.getBuffer();
   }

   public void applyBufferProcessor(YoBufferProcessor processor)
   {
      getBuffer().applyProcessor(processor);
   }

}
