package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.event.Event;
import javafx.geometry.Rectangle2D;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.SnapshotParameters;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Sphere;
import us.ihmc.euclid.Axis3D;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

public class SecondaryViewport3DManager implements SingleViewport3DManager
{
   private final Pane container;
   private final Group rootNode3D;
   private WritableImage image;
   private final ImageView cameraView;
   private final PerspectiveCamera camera;
   private final FocusBasedCameraMouseEventHandler cameraController;

   private final SnapshotParameters snapshotParameters = new SnapshotParameters();

   private final ObservedAnimationTimer animationTimer = new ObservedAnimationTimer(getClass().getSimpleName() + " - Updater")
   {
      @Override
      public void handleImpl(long now)
      {
         update(now);
      }
   };

   public SecondaryViewport3DManager(Group mainView3DRoot)
   {
      this.rootNode3D = mainView3DRoot;

      cameraView = new ImageView();
      cameraView.setPreserveRatio(true);

      // Embedding the view in the pane
      container = new Pane(cameraView);
      //      heightProperty().addListener((o, oldValue, newValue) -> cameraView.setFitHeight(newValue.doubleValue()));
      //      widthProperty().addListener((o, oldValue, newValue) -> cameraView.setFitWidth(newValue.doubleValue()));

      // Creating camera
      camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(2.0e5);

      // Setting up the camera controller.
      cameraController = new FocusBasedCameraMouseEventHandler(widthProperty(), heightProperty(), camera, Axis3D.Z, Axis3D.X);
      cameraController.enableShiftClickFocusTranslation();
      cameraView.addEventHandler(Event.ANY, cameraController);

      Sphere focusPointViz = cameraController.getFocusPointViz();
      rootNode3D.getChildren().add(focusPointViz);
      focusPointViz.visibleProperty().bind(cameraView.focusedProperty());
      MainViewport3DManager.setupNodeTrackingContextMenu(cameraController, cameraView);

      snapshotParameters.setCamera(camera);
      snapshotParameters.setDepthBuffer(true);
      snapshotParameters.setFill(Color.GRAY);

      animationTimer.start();
   }

   private void update(long now)
   {
      int height = (int) heightProperty().get();
      int width = (int) widthProperty().get();

      if (image == null || height != image.getHeight() || width != image.getWidth())
      {
         snapshotParameters.setViewport(new Rectangle2D(0, 0, width, height));
         image = new WritableImage(width, height);
         cameraView.setImage(image);
      }

      rootNode3D.snapshot(snapshotParameters, image);
   }

   @Override
   public Pane getPane()
   {
      return container;
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
      animationTimer.stop();
      cameraController.dispose();
   }

   public ReadOnlyDoubleProperty widthProperty()
   {
      return container.widthProperty();
   }

   public ReadOnlyDoubleProperty heightProperty()
   {
      return container.heightProperty();
   }
}
