package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.function.Predicate;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.MenuItem;
import javafx.scene.input.PickResult;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.Axis3D;
import us.ihmc.javaFXToolkit.cameraControllers.CameraZoomCalculator;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ContextMenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

public class MainViewport3DManager implements SingleViewport3DManager
{
   private final Pane container;
   private final SubScene subScene;
   private final Group rootNode3D;
   private final PerspectiveCamera camera;
   private final FocusBasedCameraMouseEventHandler cameraController;

   public MainViewport3DManager(Group mainView3DRoot)
   {
      rootNode3D = mainView3DRoot;

      // Creating sub-scene
      subScene = new SubScene(rootNode3D, -1, -1, true, SceneAntialiasing.BALANCED);
      subScene.setFill(Color.GRAY);
      subScene.setOnMousePressed(event -> subScene.requestFocus());

      // Embedding sub-scene in pane.
      container = new Pane(subScene);
      subScene.heightProperty().bind(heightProperty());
      subScene.widthProperty().bind(widthProperty());

      // Creating camera
      camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(2.0e5);
      subScene.setCamera(camera);

      // Setting up the camera controller.
      cameraController = new FocusBasedCameraMouseEventHandler(widthProperty(), heightProperty(), camera, Axis3D.Z, Axis3D.X);
      cameraController.enableShiftClickFocusTranslation();
      subScene.addEventHandler(Event.ANY, cameraController);

      Sphere focusPointViz = cameraController.getFocusPointViz();
      rootNode3D.getChildren().add(focusPointViz);
      focusPointViz.visibleProperty().bind(subScene.focusedProperty());
      setupNodeTrackingContextMenu(cameraController, subScene);
   }

   @Override
   public Pane getPane()
   {
      return container;
   }

   public ReadOnlyDoubleProperty widthProperty()
   {
      return container.widthProperty();
   }

   public ReadOnlyDoubleProperty heightProperty()
   {
      return container.heightProperty();
   }

   public SubScene getSubScene()
   {
      return subScene;
   }

   @Override
   public PerspectiveCamera getCamera()
   {
      return camera;
   }

   @Override
   public FocusBasedCameraMouseEventHandler getCameraController()
   {
      return cameraController;
   }

   @Override
   public void dispose()
   {
      cameraController.dispose();
   }

   // Camera controls

   public void setCameraNodeToTrack(Node nodeToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.getNodeTracker().setNodeToTrack(nodeToTrack));
   }

   public void setCameraPosition(double x, double y, double z)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.changeCameraPosition(x, y, z));
   }

   public void setCameraOrientation(double latitude, double longitude, double roll)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.getRotationCalculator().setRotation(latitude, longitude, roll));
   }

   public void setCameraFocusPosition(double x, double y, double z)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.changeFocusPosition(x, y, z, false));
   }

   public void setCameraZoom(double distanceFromFocus)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         CameraZoomCalculator zoomCalculator = cameraController.getZoomCalculator();
         if (zoomCalculator.isInvertZoomDirection())
            zoomCalculator.setZoom(-distanceFromFocus);
         else
            zoomCalculator.setZoom(distanceFromFocus);
      });
   }

   static void setupNodeTrackingContextMenu(FocusBasedCameraMouseEventHandler cameraController, Node viewport)
   {
      setupNodeTrackingContextMenu(cameraController.getNodeTracker().nodeToTrackProperty(), viewport, node -> true);
   }

   static void setupNodeTrackingContextMenu(ObjectProperty<Node> nodeTrackedProperty, Node viewport, Predicate<Node> filter)
   {
      ContextMenuTools.setupContextMenu(viewport, (owner, event) ->
      {

         PickResult pickResult = event.getPickResult();
         Node intersectedNode = pickResult.getIntersectedNode();
         if (intersectedNode == null || intersectedNode instanceof SubScene || intersectedNode == viewport || intersectedNode == nodeTrackedProperty.get()
               || !filter.test(intersectedNode))
            return null;
         MenuItem menuItem = new MenuItem("Start tracking node: " + intersectedNode.getId());
         menuItem.setOnAction(e -> nodeTrackedProperty.set(intersectedNode));
         return menuItem;
      }, (owner, event) ->
      {
         if (nodeTrackedProperty.get() == null)
            return null;
         MenuItem menuItem = new MenuItem("Stop tracking node: " + nodeTrackedProperty.get().getId());
         menuItem.setOnAction(e -> nodeTrackedProperty.set(null));
         return menuItem;
      });
   }
}
