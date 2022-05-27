package us.ihmc.scs2;

import java.io.File;
import java.net.URL;
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
import us.ihmc.scs2.sessionVisualizer.jfx.SceneVideoRecordingRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
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

   public YoSharedBuffer getBuffer()
   {
      return simulationSession.getBuffer();
   }

   public void applyBufferProcessor(YoBufferProcessor processor)
   {
      getBuffer().applyProcessor(processor);
   }

   // ------------------------------------------------------------------------------- //
   // --------------------------- Visualizer API ------------------------------------ //
   // ------------------------------------------------------------------------------- //

   /**
    * Sets the camera's orbit with respect to the focus point.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    * 
    * @param latitude  controls the look up/down angle while keeping the focus point unchanged.
    * @param longitude controls the look left/right angle while keeping the focus point unchanged.
    */
   public void setCameraOrientation(double latitude, double longitude)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraOrientation(latitude, longitude);
   }

   /**
    * Sets the camera position without moving the focus point.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    * 
    * @param x the new x-coordinate for the camera position.
    * @param y the new y-coordinate for the camera position.
    * @param z the new z-coordinate for the camera position.
    */
   public void setCameraPosition(double x, double y, double z)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraPosition(x, y, z);
   }

   /**
    * Sets the position of the focus point, i.e. what the camera is looking at.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    * 
    * @param x the new x-coordinate for the focus point.
    * @param y the new y-coordinate for the focus point.
    * @param z the new z-coordinate for the focus point.
    */
   public void setCameraFocusPosition(double x, double y, double z)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraFocusPosition(x, y, z);
   }

   /**
    * Sets the distance between the camera and focus point by moving the camera only.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    * 
    * @param distanceFromFocus the new distance between the camera and the focus point.
    */
   public void setCameraZoom(double distanceFromFocus)
   {
      if (visualizerControls != null)
         visualizerControls.setCameraZoom(distanceFromFocus);
   }

   /**
    * Requests the camera to track the rigid-body of a robot.
    * 
    * @param robotName     the name of the robot to track.
    * @param rigidBodyName the name of the body to track.
    */
   public void setCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      if (visualizerControls != null)
         visualizerControls.requestCameraRigidBodyTracking(robotName, rigidBodyName);
   }

   /**
    * Adds a static graphic to the 3D scene.
    * 
    * @param visualDefinition the visual to be added to the 3D scene.
    */
   public void addStaticVisual(VisualDefinition visualDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addStaticVisual(visualDefinition);
   }

   /**
    * Adds a collection of static graphic to the 3D scene.
    * 
    * @param visualDefinitions the collection of visuals to be added to the 3D scene.
    */
   public void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      if (visualizerControls != null)
         visualizerControls.addStaticVisuals(visualDefinitions);
   }

   /**
    * Removes a static graphic that was previously added via
    * {@link #addStaticVisual(VisualDefinition)}.
    * 
    * @param visualDefinition the visual to remove from the 3D scene.
    */
   public void removeStaticVisual(VisualDefinition visualDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.removeStaticVisual(visualDefinition);
   }

   /**
    * Removes a collection of static graphics that were previously added via
    * {@link #addStaticVisual(VisualDefinition)}.
    * 
    * @param visualDefinitions the visuals to remove from the 3D scene.
    */
   public void removeStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      if (visualizerControls != null)
         visualizerControls.removeStaticVisuals(visualDefinitions);
   }

   /**
    * Adds a dynamic graphic to the 3D scene. The new graphic is added to root group.
    * 
    * @param yoGraphicDefinition the definition of the graphic to be added.
    */
   public void addYoGraphic(YoGraphicDefinition yoGraphicDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addYoGraphic(yoGraphicDefinition);
   }

   /**
    * Adds a dynamic graphic to the 3D scene.
    * 
    * @param namespace           the desired namespace for the new graphic. The separator used is
    *                            {@value YoGraphicTools#SEPARATOR}.
    * @param yoGraphicDefinition the definition of the graphic to be added.
    */
   public void addYoGraphic(String namespace, YoGraphicDefinition yoGraphicDefinition)
   {
      if (visualizerControls != null)
         visualizerControls.addYoGraphic(namespace, yoGraphicDefinition);
   }

   /**
    * Adds a variable entry to the default entry tab.
    * 
    * @param variableName the name of the variable to add. The variable will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    */
   public void addYoEntry(String variableName)
   {
      if (visualizerControls != null)
         visualizerControls.addYoEntry(variableName);
   }

   /**
    * Adds variable entries to the default entry tab.
    * 
    * @param variableNames the name of the variables to add. The variables will be looked up using
    *                      {@link YoRegistry#findVariable(String)}.
    */
   public void addYoEntry(Collection<String> variableNames)
   {
      if (visualizerControls != null)
         visualizerControls.addYoEntry(variableNames);
   }

   /**
    * Adds a variable entry to the entry tab named {@code groupName}. The tab will be created if it
    * doesn't exist yet.
    * 
    * @param groupName    the name of the tab.
    * @param variableName the name of the variable to add. The variable will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    */
   public void addYoEntry(String groupName, String variableName)
   {
      if (visualizerControls != null)
         visualizerControls.addYoEntry(groupName, variableName);
   }

   /**
    * Adds variable entries to the entry tab named {@code groupName}. The tab will be created if it
    * doesn't exist yet.
    * 
    * @param groupName    the name of the tab.
    * @param variableName the name of the variables to add. The variables will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    */
   public void addYoEntry(String groupName, Collection<String> variableNames)
   {
      if (visualizerControls != null)
         visualizerControls.addYoEntry(groupName, variableNames);
   }

   /**
    * Captures a video of the 3D scene from the playback data.
    * <p>
    * The file extension should be {@value SessionVisualizerIOTools#videoFileExtension}.
    * </p>
    * 
    * @param file the target file where the video is to be written.
    */
   public void exportVideo(File file)
   {
      if (visualizerControls != null)
         visualizerControls.exportVideo(file);
   }

   /**
    * Captures a video of the 3D scene from the playback data.
    * 
    * @param request the request.
    */
   public void exportVideo(SceneVideoRecordingRequest request)
   {
      if (visualizerControls != null)
         visualizerControls.exportVideo(request);
   }

   /**
    * Disables GUI controls. Can be used to prevent the user from interfering with a background process
    * temporarily.
    */
   public void disableGUIComponents()
   {
      if (visualizerControls != null)
         visualizerControls.disableUserControls();
   }

   /**
    * Enables GUI controls.
    */
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

   /**
    * Causes the caller's thread to pause until the visualizer is fully operational.
    */
   public void waitUntilVisualizerIsFullyUp()
   {
      if (visualizerControls != null)
         visualizerControls.waitUntilFullyUp();
   }

   /**
    * Causes the caller's thread to pause until the visualizer is fully operational.
    */
   public void waitUntilVisualizerIsDown()
   {
      if (visualizerControls != null)
         visualizerControls.waitUntilDown();
   }

   // TODO Missing setupGraph, setupGraphGroup

}
