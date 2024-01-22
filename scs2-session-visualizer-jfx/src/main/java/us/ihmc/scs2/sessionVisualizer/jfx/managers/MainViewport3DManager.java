package us.ihmc.scs2.sessionVisualizer.jfx.managers;

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
import us.ihmc.scs2.definition.camera.YoLevelOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.camera.YoOrbitalCoordinateDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionPropertiesHelper;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.Camera3DOptionsPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraControlMode;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler.TrackingTargetType;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraOrbitHandler;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.LevelOrbitalCoordinateProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.OrbitalCoordinateProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.PerspectiveCameraController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

import java.io.IOException;
import java.util.function.Predicate;

public class MainViewport3DManager implements SingleViewport3DManager
{
   /** The viewport background color when there is no skybox. */
   public static final String VIEWPORT_BACKGROUND_COLOR = SessionPropertiesHelper.loadStringPropertyOrEnvironment("scs2.session.gui.viewport.background",
                                                                                                                  "SCS2_GUI_VIEWPORT_BACKGROUND",
                                                                                                                  "gray");
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
      subScene.setFill(Color.web(VIEWPORT_BACKGROUND_COLOR));
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
      cameraController.start();
      subScene.addEventHandler(Event.ANY, cameraController);

      Sphere focusPointViz = cameraController.getFocalPointViz();
      if (focusPointViz != null)
      {
         rootNode3D.getChildren().add(focusPointViz);
         focusPointViz.visibleProperty().bind(subScene.focusedProperty());
      }
      setupContextMenu(cameraController, yoCompositeSearchManager, yoManager, referenceFrameManager, subScene);
   }

   @Override
   public void startSession(Session session)
   {
   }

   @Override
   public void stopSession()
   {
      cameraController.cameraPositionCoordinatesToTrackProperty().setValue(null);
      cameraController.cameraOrbitalCoordinatesToTrackProperty().setValue(null);
      cameraController.cameraLevelOrbitalCoordinatesToTrackProperty().setValue(null);
      cameraController.getFocalPointHandler().coordinatesToTrackProperty().set(null);
      cameraController.getFocalPointHandler().nodeToTrackProperty().set(null);
   }

   @Override
   public boolean isSessionLoaded()
   {
      return true;
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
      cameraController.stop();
   }

   // Camera controls

   public void setCameraFocalTargetTypeToTrack(TrackingTargetType targetType)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.getFocalPointHandler().targetTypeProperty().set(targetType));
   }

   public void setCameraFocalNodeToTrack(Node nodeToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         cameraController.getFocalPointHandler().nodeToTrackProperty().set(nodeToTrack);
      });
   }

   public void setCameraFocalPositionToTrack(YoTuple3DDefinition coordinatesToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         Tuple3DProperty coordinatesProperty;

         try
         {
            coordinatesProperty = CompositePropertyTools.toTuple3DProperty(yoManager.getRootRegistryDatabase(), referenceFrameManager, coordinatesToTrack);
         }
         catch (Exception e)
         {
            // Print stack-trace and cancel operation
            e.printStackTrace();
            return;
         }
         cameraController.getFocalPointHandler().coordinatesToTrackProperty().setValue(coordinatesProperty);
      });
   }

   public void setCameraPositionToTrack(YoTuple3DDefinition coordinatesToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         Tuple3DProperty coordinatesProperty;

         try
         {
            coordinatesProperty = CompositePropertyTools.toTuple3DProperty(yoManager.getRootRegistryDatabase(), referenceFrameManager, coordinatesToTrack);
         }
         catch (Exception e)
         {
            // Print stack-trace and cancel operation
            e.printStackTrace();
            return;
         }
         cameraController.cameraPositionCoordinatesToTrackProperty().setValue(coordinatesProperty);
      });
   }

   public void setCameraOrbitToTrack(YoOrbitalCoordinateDefinition coordinatesToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         OrbitalCoordinateProperty coordinatesProperty;

         try
         {
            coordinatesProperty = new OrbitalCoordinateProperty(CompositePropertyTools.toCompositeProperty(yoManager.getRootRegistryDatabase(),
                                                                                                           referenceFrameManager,
                                                                                                           coordinatesToTrack));
         }
         catch (Exception e)
         {
            // Print stack-trace and cancel operation
            e.printStackTrace();
            return;
         }
         cameraController.cameraOrbitalCoordinatesToTrackProperty().setValue(coordinatesProperty);
      });
   }

   public void setCameraLevelOrbitToTrack(YoLevelOrbitalCoordinateDefinition coordinatesToTrack)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         LevelOrbitalCoordinateProperty coordinatesProperty;

         try
         {
            coordinatesProperty = new LevelOrbitalCoordinateProperty(CompositePropertyTools.toCompositeProperty(yoManager.getRootRegistryDatabase(),
                                                                                                                referenceFrameManager,
                                                                                                                coordinatesToTrack));
         }
         catch (Exception e)
         {
            // Print stack-trace and cancel operation
            e.printStackTrace();
            return;
         }
         cameraController.cameraLevelOrbitalCoordinatesToTrackProperty().setValue(coordinatesProperty);
      });
   }

   public void setCameraPosition(double x, double y, double z)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.setCameraPosition(x, y, z));
   }

   public void setCameraOrientation(double latitude, double longitude, double roll)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.getOrbitHandler().setRotation(longitude, latitude, roll));
   }

   public void setCameraFocalPosition(double x, double y, double z)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.setFocalPoint(x, y, z, false));
   }

   public void setCameraControlMode(CameraControlMode cameraControlMode)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> cameraController.cameraControlMode().setValue(cameraControlMode));
   }

   public void setCameraZoom(double distanceFromFocus)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         CameraOrbitHandler orbitHandler = cameraController.getOrbitHandler();
         orbitHandler.distanceProperty().set(distanceFromFocus);
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
      CameraFocalPointHandler focalPointHandler = cameraController.getFocalPointHandler();
      ObjectProperty<Node> nodeToTrackProperty = focalPointHandler.nodeToTrackProperty();

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
                              });
         return menuItem;
      }, (owner, event) ->
                                 {
                                    if (nodeToTrackProperty.get() == null || focalPointHandler.isTrackingDisabled())
                                       return null;
                                    MenuItem menuItem = new MenuItem("Stop tracking node: " + nodeToTrackProperty.get().getId());
                                    menuItem.setOnAction(e ->
                                                         {
                                                            nodeToTrackProperty.set(null);
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
            {
               window.close();
               controller.closeAndDispose();
            }
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
