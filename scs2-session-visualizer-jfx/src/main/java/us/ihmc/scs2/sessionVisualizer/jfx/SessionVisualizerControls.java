package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;
import java.net.URL;
import java.util.Collection;
import java.util.Collections;

import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.layout.Pane;
import javafx.stage.Window;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoEntry.YoEntryListDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface SessionVisualizerControls
{
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
   void setCameraOrientation(double latitude, double longitude);

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
   void setCameraPosition(double x, double y, double z);

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
   void setCameraFocusPosition(double x, double y, double z);

   /**
    * Sets the distance between the camera and focus point by moving the camera only.
    * <p>
    * The camera is using orbit controls, i.e. the camera is always looking at a target and easily
    * rotate around that target.
    * </p>
    * 
    * @param distanceFromFocus the new distance between the camera and the focus point.
    */
   void setCameraZoom(double distanceFromFocus);

   /**
    * Requests the camera to track the rigid-body of a robot.
    * 
    * @param robotName     the name of the robot to track.
    * @param rigidBodyName the name of the body to track.
    */
   default void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      waitUntilFullyUp();
      submitMessage(getTopics().getCameraTrackObject(), new CameraObjectTrackingRequest(robotName, rigidBodyName));
   }

   /**
    * Adds a collection of static graphic to the 3D scene.
    * 
    * @param visualDefinitions the collection of visuals to be added to the 3D scene.
    */
   default void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         addStaticVisual(visualDefinition);
      }
   }

   /**
    * Adds a static graphic to the 3D scene.
    * 
    * @param visualDefinition the visual to be added to the 3D scene.
    */
   void addStaticVisual(VisualDefinition visualDefinition);

   /**
    * Removes a collection of static graphics that were previously added via
    * {@link #addStaticVisual(VisualDefinition)}.
    * 
    * @param visualDefinitions the visuals to remove from the 3D scene.
    */
   default void removeStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      for (VisualDefinition visualDefinition : visualDefinitions)
      {
         removeStaticVisual(visualDefinition);
      }
   }

   /**
    * Removes a static graphic that was previously added via
    * {@link #addStaticVisual(VisualDefinition)}.
    * 
    * @param visualDefinition the visual to remove from the 3D scene.
    */
   void removeStaticVisual(VisualDefinition visualDefinition);

   /**
    * Adds a dynamic graphic to the 3D scene.
    * 
    * @param namespace           the desired namespace for the new graphic. The separator used is
    *                            {@value YoGraphicTools#SEPARATOR}.
    * @param yoGraphicDefinition the definition of the graphic to be added.
    */
   default void addYoGraphic(String namespace, YoGraphicDefinition yoGraphicDefinition)
   {
      String[] subNames = namespace.split(YoGraphicTools.SEPARATOR);
      if (subNames == null || subNames.length == 0)
      {
         addYoGraphic(yoGraphicDefinition);
      }
      else
      {
         for (int i = subNames.length - 1; i >= 0; i--)
         {
            yoGraphicDefinition = new YoGraphicGroupDefinition(subNames[i], yoGraphicDefinition);
         }

         addYoGraphic(yoGraphicDefinition);
      }
   }

   /**
    * Adds a dynamic graphic to the 3D scene. The new graphic is added to root group.
    * 
    * @param yoGraphicDefinition the definition of the graphic to be added.
    */
   default void addYoGraphic(YoGraphicDefinition yoGraphicDefinition)
   {
      submitMessage(getTopics().getAddYoGraphicRequest(), yoGraphicDefinition);
   }

   /**
    * Adds a variable entry to the default entry tab.
    * 
    * @param variableName the name of the variable to add. The variable will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    */
   default void addYoEntry(String variableName)
   {
      addYoEntry(Collections.singletonList(variableName));
   }

   /**
    * Adds variable entries to the default entry tab.
    * 
    * @param variableNames the name of the variables to add. The variables will be looked up using
    *                      {@link YoRegistry#findVariable(String)}.
    */
   default void addYoEntry(Collection<String> variableNames)
   {
      addYoEntry(null, variableNames);
   }

   /**
    * Adds a variable entry to the entry tab named {@code groupName}. The tab will be created if it
    * doesn't exist yet.
    * 
    * @param groupName    the name of the tab.
    * @param variableName the name of the variable to add. The variable will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    */
   default void addYoEntry(String groupName, String variableName)
   {
      addYoEntry(groupName, Collections.singletonList(variableName));
   }

   /**
    * Adds variable entries to the entry tab named {@code groupName}. The tab will be created if it
    * doesn't exist yet.
    * 
    * @param groupName    the name of the tab.
    * @param variableName the name of the variables to add. The variables will be looked up using
    *                     {@link YoRegistry#findVariable(String)}.
    */
   default void addYoEntry(String groupName, Collection<String> variableNames)
   {
      submitMessage(getTopics().getYoEntryListAdd(), YoEntryListDefinition.newYoVariableEntryList(groupName, variableNames));
   }

   /**
    * Captures a video of the 3D scene from the playback data.
    * <p>
    * The file extension should be {@value SessionVisualizerIOTools#videoFileExtension}.
    * </p>
    * 
    * @param file the target file where the video is to be written.
    */
   default void exportVideo(File file)
   {
      SceneVideoRecordingRequest request = new SceneVideoRecordingRequest();
      request.setFile(file);
      exportVideo(request);
   }

   /**
    * Captures a video of the 3D scene from the playback data.
    * 
    * @param request the request.
    */
   void exportVideo(SceneVideoRecordingRequest request);

   /**
    * Disables GUI controls. Can be used to prevent the user from interfering with a background process
    * temporarily.
    */
   default void disableUserControls()
   {
      submitMessage(getTopics().getDisableUserControls(), true);
   }

   /**
    * Enables GUI controls.
    */
   default void enableUserControls()
   {
      submitMessage(getTopics().getDisableUserControls(), false);
   }

   /**
    * Gets the messager's topics.
    * <p>
    * The visualizer relies on the {@link Messager} framework to communicate requests.
    * </p>
    * 
    * @return the topics this visualizer uses.
    */
   SessionVisualizerTopics getTopics();

   /**
    * Submits a message.
    * 
    * @param <T>            the type of the message content imposed by the selected topic.
    * @param topic          the topic to with the message is to be submitted.
    * @param messageContent the content of the message.
    */
   <T> void submitMessage(Topic<T> topic, T messageContent);

   /**
    * Gets the main window's instance.
    * 
    * @return the main window.
    */
   Window getPrimaryWindow();

   /**
    * Adds a custom JavaFX control, for instance a {@link Button}, which is displayed in the user side
    * panel on the right side of the main window.
    * 
    * @param control the custom control to add.
    */
   void addCustomControl(Node control);

   /**
    * Removes a custom JavaFX control that was previously added via {@link #addCustomControl(Node)}.
    * 
    * @param control the control to be removed.
    * @return whether the control was found and removed successfully.
    */
   boolean removeCustomControl(Node control);

   /**
    * Loads and adds a mini-GUI from an FXML file. The GUI is displayed in the user side panel on the
    * right side of the main window.
    * 
    * @param name         the title of the new pane.
    * @param fxmlResource the locator to the FXML resource.
    */
   void loadCustomPane(String name, URL fxmlResource);

   /**
    * Adds a mini-GUI to the user side panel on the right side of the main window.
    * 
    * @param name the title of the new pane.
    * @param pane the pane to be added.
    */
   void addCustomPane(String name, Pane pane);

   /**
    * Removes a pane previously added via {@link #loadCustomPane(String, URL)} or
    * {@link #addCustomPane(String, Pane)}.
    * 
    * @param name the title of the pane to remove.
    */
   boolean removeCustomPane(String name);

   /**
    * Requests to "gently" shutdown the visualizer and the session.
    * <p>
    * A confirmation dialog may popup to confirm the shutdown and ask for saving the GUI configuration.
    * </p>
    * <p>
    * After shutdown, the visualizer and the session become useless.
    * </p>
    */
   void shutdown();

   /**
    * Requests an immediate shutdown of the visualizer and the session.
    * <p>
    * No confirmation dialog will eb shown and the GUI configuration is not saved.
    * </p>
    * <p>
    * After shutdown, the visualizer and the session become useless.
    * </p>
    */
   void shutdownNow();

   /**
    * Adds a listener to be notified when the visualizer just shutdown.
    * 
    * @param listener the shutdown listener.
    */
   void addVisualizerShutdownListener(Runnable listener);

   /**
    * Causes the caller's thread to pause until the visualizer is fully operational.
    */
   void waitUntilFullyUp();

   /**
    * Causes the caller's thread to pause until the visualizer gets shutdown.
    */
   void waitUntilDown();
}
