package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.io.IOException;
import java.util.function.Predicate;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.event.Event;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.MenuItem;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.PickResult;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Sphere;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import us.ihmc.euclid.Axis3D;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.Camera3DOptionsPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler.TrackingTargetType;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointTargetTracker;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraZoomCalculator;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.PerspectiveCameraController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

public class MainViewport3DManager implements SingleViewport3DManager
{
   private final Pane container;
   private final SubScene subScene;
   private final Group rootNode3D;
   private final YoManager yoManager;
   private final ReferenceFrameManager referenceFrameManager;

   private final PerspectiveCamera camera;
   private final PerspectiveCameraController cameraController;

   public MainViewport3DManager(Group mainView3DRoot,
                                YoManager yoManager,
                                YoCompositeSearchManager yoCompositeSearchManager,
                                ReferenceFrameManager referenceFrameManager)
   {
      rootNode3D = mainView3DRoot;
      this.yoManager = yoManager;
      this.referenceFrameManager = referenceFrameManager;

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
      cameraController = new PerspectiveCameraController(widthProperty(), heightProperty(), camera, Axis3D.Z, Axis3D.X);
      cameraController.enableShiftClickFocusTranslation();
      subScene.addEventHandler(Event.ANY, cameraController);

      Sphere focusPointViz = cameraController.getFocusPointViz();
      if (focusPointViz != null)
      {
         rootNode3D.getChildren().add(focusPointViz);
         focusPointViz.visibleProperty().bind(subScene.focusedProperty());
      }
      setupContextMenu(cameraController, yoCompositeSearchManager, yoManager, referenceFrameManager, subScene);
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
   public PerspectiveCameraController getCameraController()
   {
      return cameraController;
   }

   @Override
   public void dispose()
   {
      cameraController.dispose();
   }

   // Camera controls

   public void setCameraTargetTypeToTrack(TrackingTargetType targetType)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.getTargetTracker().targetTypeProperty().set(targetType));
   }

   public void setCameraNodeToTrack(Node nodeToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         cameraController.getTargetTracker().nodeToTrackProperty().set(nodeToTrack);
         cameraController.getTargetTracker().targetTypeProperty().set(TrackingTargetType.Node);
      });
   }

   public void setCameraCoordinatesToTrack(YoTuple3DDefinition coordinatesToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         Tuple3DProperty tuple3DProperty;

         try
         {
            tuple3DProperty = CompositePropertyTools.toTuple3DProperty(yoManager.getRootRegistryDatabase(), referenceFrameManager, coordinatesToTrack);
         }
         catch (Exception e)
         {
            // Print stack-trace and cancel operation
            e.printStackTrace();
            return;
         }
         cameraController.getTargetTracker().coordinatesToTrackProperty().setValue(tuple3DProperty);
         cameraController.getTargetTracker().targetTypeProperty().set(TrackingTargetType.YoCoordinates);
      });
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
         if (zoomCalculator.invertZoomDirectionProperty().get())
            zoomCalculator.zoomProperty().set(-distanceFromFocus);
         else
         {
            final double zoom = distanceFromFocus;
            zoomCalculator.zoomProperty().set(zoom);
         }
      });
   }

   static void setupContextMenu(PerspectiveCameraController cameraController,
                                YoCompositeSearchManager searchManager,
                                YoManager yoManager,
                                ReferenceFrameManager referenceFrameManager,
                                Node viewport)
   {
      setupContextMenu(cameraController, searchManager, yoManager, referenceFrameManager, viewport, node -> true);
   }

   static void setupContextMenu(PerspectiveCameraController cameraController,
                                YoCompositeSearchManager searchManager,
                                YoManager yoManager,
                                ReferenceFrameManager referenceFrameManager,
                                Node viewport,
                                Predicate<Node> filter)
   {
      CameraFocalPointTargetTracker targetTracker = cameraController.getTargetTracker();
      ObjectProperty<TrackingTargetType> targetTypeProperty = targetTracker.targetTypeProperty();
      ObjectProperty<Node> nodeToTrackProperty = targetTracker.nodeToTrackProperty();

      MenuTools.setupContextMenu(viewport, (owner, event) ->
      {

         PickResult pickResult = event.getPickResult();
         Node intersectedNode = pickResult.getIntersectedNode();
         if (intersectedNode == null || intersectedNode instanceof SubScene || intersectedNode == viewport || intersectedNode == nodeToTrackProperty.get()
             || !filter.test(intersectedNode))
            return null;
         MenuItem menuItem = new MenuItem("Start tracking node: " + intersectedNode.getId());
         menuItem.setOnAction(e ->
         {
            nodeToTrackProperty.set(intersectedNode);
            targetTypeProperty.set(TrackingTargetType.Node);
         });
         return menuItem;
      }, (owner, event) ->
      {
         if (nodeToTrackProperty.get() == null)
            return null;
         MenuItem menuItem = new MenuItem("Stop tracking node: " + nodeToTrackProperty.get().getId());
         menuItem.setOnAction(e ->
         {
            nodeToTrackProperty.set(null);
            targetTypeProperty.set(TrackingTargetType.Disabled);
         });
         return menuItem;
      }, (owner, event) ->
      {
         MenuItem menuItem = new MenuItem("Camera properties...");
         menuItem.setOnAction(e -> openCameraPropertiesDialog(cameraController,
                                                              searchManager,
                                                              yoManager.getLinkedRootRegistry(),
                                                              referenceFrameManager,
                                                              viewport));
         return menuItem;
      });
   }

   static void openCameraPropertiesDialog(PerspectiveCameraController cameraController,
                                          YoCompositeSearchManager searchManager,
                                          LinkedYoRegistry linkedRootRegistry,
                                          ReferenceFrameManager referenceFrameManager,
                                          Node viewport)
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CAMERA3D_OPTIONS_PANE_URL);
         Pane rootPane = loader.load();
         Camera3DOptionsPaneController controller = loader.getController();
         controller.initialize(cameraController, searchManager, linkedRootRegistry, referenceFrameManager);

         Stage window = new Stage(StageStyle.UTILITY);
         window.setScene(new Scene(rootPane));
         window.addEventHandler(KeyEvent.KEY_PRESSED, e ->
         {
            if (e.getCode() == KeyCode.ESCAPE)
               window.close();
         });
         Window owner = viewport.getScene().getWindow();
         owner.addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
         {
            if (!e.isConsumed())
               window.close();
         });
         window.initOwner(owner);
         window.setTitle("Camera 3D options");
         window.setOpacity(0.0);
         window.toFront();
         window.show();
         Timeline timeline = new Timeline();
         KeyFrame key = new KeyFrame(Duration.seconds(0.125), new KeyValue(window.opacityProperty(), 1.0));
         timeline.getKeyFrames().add(key);
         timeline.play();

         //         window.setOnHidden(e -> stop());
         //         window.setOnShowing(e -> start());

      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
