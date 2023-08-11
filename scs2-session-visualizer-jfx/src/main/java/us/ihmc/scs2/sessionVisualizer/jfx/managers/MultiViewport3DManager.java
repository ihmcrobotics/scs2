package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import us.ihmc.scs2.sessionVisualizer.jfx.Camera3DRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.Camera3DRequest.CameraControlRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.Camera3DRequest.FocalPointRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler.TrackingTargetType;

public class MultiViewport3DManager
{
   private final GridPane container = new GridPane();
   private final Group mainView3DRoot;

   private final MainViewport3DManager mainViewport;
   private final YoManager yoManager;
   private final YoCompositeSearchManager yoCompositeSearchManager;
   private final ReferenceFrameManager referenceFrameManager;

   private final ObservableList<SingleViewport3DManager> allViewports = FXCollections.observableArrayList();
   private final IntegerProperty numberOfColumns = new SimpleIntegerProperty(this, "numberOfColumns", 2);

   public MultiViewport3DManager(Group mainView3DRoot,
                                 YoManager yoManager,
                                 YoCompositeSearchManager yoCompositeSearchManager,
                                 ReferenceFrameManager referenceFrameManager)
   {
      this.mainView3DRoot = mainView3DRoot;
      this.yoManager = yoManager;
      this.yoCompositeSearchManager = yoCompositeSearchManager;
      this.referenceFrameManager = referenceFrameManager;

      allViewports.addListener((ListChangeListener<SingleViewport3DManager>) change -> refreshLayout());
      numberOfColumns.addListener((o, oldValue, newValue) -> refreshLayout());

      mainViewport = new MainViewport3DManager(mainView3DRoot, yoManager, yoCompositeSearchManager, referenceFrameManager);
      allViewports.add(mainViewport);
   }

   // TODO Only available for the main viewport for now, need to expand this.
   public void submitRequest(Camera3DRequest request)
   {
      FocalPointRequest focalPointRequest = request.getFocalPointRequest();

      if (focalPointRequest != null && focalPointRequest.getTrackingTargetType() != null)
      {
         switch (focalPointRequest.getTrackingTargetType())
         {
            case Node:
               mainViewport.setCameraFocalNodeToTrack(focalPointRequest.getNode());
               break;
            case YoCoordinates:
               mainViewport.setCameraFocalPositionToTrack(focalPointRequest.getCoordinatesToTrack());
               break;
            case Disabled:
               mainViewport.setCameraFocalTargetTypeToTrack(TrackingTargetType.Disabled);
               break;
            default:
               throw new IllegalStateException("Unexpected target type: " + focalPointRequest.getTrackingTargetType());
         }
      }

      CameraControlRequest cameraControlRequest = request.getCameraControlRequest();

      if (cameraControlRequest != null && cameraControlRequest.getControlMode() != null)
      {
         switch (cameraControlRequest.getControlMode())
         {
            case Position:
               mainViewport.setCameraPositionToTrack(cameraControlRequest.getPositionToTrack());
               break;
            case Orbital:
               mainViewport.setCameraOrbitToTrack(cameraControlRequest.getOrbitToTrack());
               break;
            case LevelOrbital:
               mainViewport.setCameraLevelOrbitToTrack(cameraControlRequest.getLevelOrbitToTrack());
               break;
            default:
               throw new IllegalStateException("Unexpected control mode: " + cameraControlRequest.getControlMode());
         }
      }
   }

   public void refreshLayout()
   {
      container.getChildren().clear();

      int numCols = numberOfColumns.get();

      int col = 0;
      int row = 0;

      for (int i = 0; i < allViewports.size(); i++)
      {
         Pane pane = allViewports.get(i).getPane();
         container.add(pane, col, row);
         GridPane.setHgrow(pane, Priority.SOMETIMES);
         GridPane.setVgrow(pane, Priority.SOMETIMES);
         col++;
         if (col >= numCols)
         {
            col = 0;
            row++;
         }
      }
   }

   public void addSecondaryViewport()
   {
      allViewports.add(new SecondaryViewport3DManager(mainView3DRoot, yoManager, yoCompositeSearchManager, referenceFrameManager));
   }

   public MainViewport3DManager getMainViewport()
   {
      return mainViewport;
   }

   public Pane getPane()
   {
      return container;
   }

   public void dispose()
   {
      allViewports.forEach(viewport -> viewport.dispose());
      allViewports.clear();
   }
}
