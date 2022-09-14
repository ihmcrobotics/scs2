package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.File;
import java.net.URL;
import java.util.Collection;
import java.util.Collections;

import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.Pane;
import javafx.stage.Window;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoBooleanProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoEnumAsStringProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoIntegerProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoLongProperty;
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
   void requestCameraRigidBodyTracking(String robotName, String rigidBodyName);

   /**
    * Requests to show the overhead view next to the 3D viewport where 2D graphics are displayed.
    * 
    * @param show whether the plotter 2D should be visible or not.
    */
   void showOverheadPlotter2D(boolean show);

   /**
    * Requests the plotter to track the {@code YoVariable} coordinates.
    * 
    * @param xVariableName the name (fullname or short name) of the {@code YoVariable} to track. Can be
    *                      a double value.
    * @param yVariableName the name (fullname or short name) of the {@code YoVariable} to track. Can be
    *                      a double value.
    * @param frameName     the name of the reference frame in which the coordinates are expressed. If
    *                      {@code null}, world frame is used.
    */
   void requestPlotter2DCoordinateTracking(String xVariableName, String yVariableName, String frameName);

   /**
    * Adds a static graphic to the 3D scene.
    * 
    * @param visualDefinition the visual to be added to the 3D scene.
    */
   void addStaticVisual(VisualDefinition visualDefinition);

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
    * Removes a static graphic that was previously added via
    * {@link #addStaticVisual(VisualDefinition)}.
    * 
    * @param visualDefinition the visual to remove from the 3D scene.
    */
   void removeStaticVisual(VisualDefinition visualDefinition);

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
    * Adds a dynamic graphic to the 3D scene. The new graphic is added to root group.
    * 
    * @param yoGraphicDefinition the definition of the graphic to be added.
    * @see SCS1GraphicConversionTools
    */
   void addYoGraphic(YoGraphicDefinition yoGraphicDefinition);

   /**
    * Adds dynamic graphics to the 3D scene. The new graphics are added to root group.
    * 
    * @param yoGraphicDefinitions the definitions of the graphics to be added.
    * @see SCS1GraphicConversionTools
    */
   default void addYoGraphics(Collection<? extends YoGraphicDefinition> yoGraphicDefinitions)
   {
      for (YoGraphicDefinition yoGraphicDefinition : yoGraphicDefinitions)
      {
         addYoGraphic(yoGraphicDefinition);
      }
   }

   /**
    * Adds a dynamic graphic to the 3D scene.
    * 
    * @param namespace           the desired namespace for the new graphic. The separator used is
    *                            {@value YoGraphicTools#SEPARATOR}.
    * @param yoGraphicDefinition the definition of the graphic to be added.
    * @see SCS1GraphicConversionTools
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
    * Adds dynamic graphics to the 3D scene.
    * 
    * @param namespace            the desired namespace for the new graphics. The separator used is
    *                             {@value YoGraphicTools#SEPARATOR}.
    * @param yoGraphicDefinitions the definitions of the graphics to be added.
    * @see SCS1GraphicConversionTools
    */
   default void addYoGraphic(String namespace, Collection<? extends YoGraphicDefinition> yoGraphicDefinitions)
   {
      String[] subNames = namespace.split(YoGraphicTools.SEPARATOR);
      if (subNames == null || subNames.length == 0)
      {
         addYoGraphics(yoGraphicDefinitions);
      }
      else
      {
         YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(subNames[subNames.length - 1], yoGraphicDefinitions);

         for (int i = subNames.length - 2; i >= 0; i--)
         {
            group = new YoGraphicGroupDefinition(subNames[i], group);
         }

         addYoGraphic(group);
      }
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
   void addYoEntry(String groupName, Collection<String> variableNames);

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
   void disableGUIControls();

   /**
    * Enables GUI controls.
    */
   void enableGUIControls();

   /**
    * Gets the main window's instance.
    * 
    * @return the main window.
    */
   Window getPrimaryGUIWindow();

   /**
    * Adds a custom JavaFX control, for instance a {@link Button}, which is displayed in the user side
    * panel on the right side of the main window.
    * 
    * @param control the custom control to add.
    */
   void addCustomGUIControl(Node control);

   /**
    * Removes a custom JavaFX control that was previously added via {@link #addCustomGUIControl(Node)}.
    * 
    * @param control the control to be removed.
    * @return whether the control was found and removed successfully.
    */
   boolean removeCustomGUIControl(Node control);

   /**
    * Loads and adds a mini-GUI from an FXML file. The GUI is displayed in the user side panel on the
    * right side of the main window.
    * 
    * @param name         the title of the new pane.
    * @param fxmlResource the locator to the FXML resource.
    */
   void loadCustomGUIPane(String name, URL fxmlResource);

   /**
    * Adds a mini-GUI to the user side panel on the right side of the main window.
    * 
    * @param name the title of the new pane.
    * @param pane the pane to be added.
    */
   void addCustomGUIPane(String name, Pane pane);

   /**
    * Removes a pane previously added via {@link #loadCustomGUIPane(String, URL)} or
    * {@link #addCustomGUIPane(String, Pane)}.
    * 
    * @param name the title of the pane to remove.
    */
   boolean removeCustomGUIPane(String name);

   /**
    * Creates a property that is bound to the {@code YoDouble} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoDoubleProperty#get()} and to safely submit a new value via
    * {@link YoDoubleProperty#set(double)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    * 
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *         variable doesn't match, or there is no active session.
    */
   YoDoubleProperty newYoDoubleProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoInteger} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoIntegerProperty#get()} and to safely submit a new value via
    * {@link YoIntegerProperty#set(int)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    * 
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *         variable doesn't match, or there is no active session.
    */
   YoIntegerProperty newYoIntegerProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoLong} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoLongProperty#get()} and to safely submit a new value via
    * {@link YoLongProperty#set(long)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    * 
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *         variable doesn't match, or there is no active session.
    */
   YoLongProperty newYoLongProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoBoolean} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoBooleanProperty#get()} and to safely submit a new value via
    * {@link YoBooleanProperty#set(boolean)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    * 
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *         variable doesn't match, or there is no active session.
    */
   YoBooleanProperty newYoBooleanProperty(String variableName);

   /**
    * Creates a property that is bound to the {@code YoEnum<E>} of the given name.
    * <p>
    * The new property can be used to safely access the current value of the variable via
    * {@link YoEnumAsStringProperty#get()} and to safely submit a new value via
    * {@link YoEnumAsStringProperty#set(E)}.
    * </p>
    * <p>
    * The new property can be bound to UI control like a {@link ToggleButton} for instance.
    * </p>
    * <p>
    * A session has to be active to be able to create a new property.
    * </p>
    * 
    * @param variableName the name of the variable to create the property for.
    * @return the new property or {@code null} if the variable could not be found, the type of the
    *         variable doesn't match, or there is no active session.
    */
   <E extends Enum<E>> YoEnumAsStringProperty<E> newYoEnumProperty(String variableName);

   /**
    * Adds a listener to be notified when the active session has changed.
    * <p>
    * This is particularly useful for initializing controls once the session has changed.
    * </p>
    * 
    * @param listener the session changed listener.
    */
   void addSessionChangedListener(SessionChangeListener listener);

   /**
    * Removes a listener previously added.
    * 
    * @param listener the listener to remove.
    * @return {@code true} if the listener was successfully removed.
    */
   boolean removeSessionChangedListener(SessionChangeListener listener);

   /**
    * Requests to "gently" shutdown the visualizer and the session.
    * <p>
    * A confirmation dialog may popup to confirm the shutdown and ask for saving the GUI configuration.
    * </p>
    * <p>
    * After shutdown, the visualizer and the session become useless.
    * </p>
    */
   void requestVisualizerShutdown();

   /**
    * Requests an immediate shutdown of the visualizer and the session.
    * <p>
    * No confirmation dialog will be shown and the GUI configuration is not saved.
    * </p>
    * <p>
    * After shutdown, the visualizer and the session become useless.
    * </p>
    */
   void shutdownSession();

   /**
    * Indicates whether the visualizer has been shutdown or not.
    * 
    * @return {@code true} if the visualizer has been shutdown, {@code false} otherwise.
    */
   boolean isVisualizerShutdown();

   /**
    * Adds a listener to be notified when the visualizer just shutdown.
    * 
    * @param listener the shutdown listener.
    */
   void addVisualizerShutdownListener(Runnable listener);

   /**
    * Causes the caller's thread to pause until the visualizer is fully operational.
    */
   void waitUntilVisualizerFullyUp();

   /**
    * Causes the caller's thread to pause until the visualizer gets shutdown.
    */
   void waitUntilVisualizerDown();
}
