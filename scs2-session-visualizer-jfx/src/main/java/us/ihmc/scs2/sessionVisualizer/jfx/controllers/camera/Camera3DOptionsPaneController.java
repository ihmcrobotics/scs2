package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleGroup;
import javafx.scene.control.Tooltip;
import javafx.scene.image.ImageView;
import javafx.scene.transform.Transform;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.CameraFocalPointHandler.TrackingTargetType;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.DoubleSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

public class Camera3DOptionsPaneController
{
   @FXML
   private RadioButton trackCoordinatesButton, trackNodeButton;
   @FXML
   private YoCompositeEditorPaneController yoCoordinateEditorController;
   @FXML
   private TextField trackingNodeTextField;
   @FXML
   private TextField xFocalPointCurrentTextField, yFocalPointCurrentTextField, zFocalPointCurrentTextField;

   private enum CameraPositionType
   {
      Position, Orbital, LevelOrbital
   };

   @FXML
   private ComboBox<CameraPositionType> cameraPositionComboxBox;

   // The following controls are for either:
   //  - x, y, z
   //  - distance, yaw, pitch
   //  - distance, yaw, z
   /** Label using for either "x" or "distance". */
   @FXML
   private Label cameraLabel1;
   /** Label using for either "y" or "yaw". */
   @FXML
   private Label cameraLabel2;
   /** Label using for either "z" or "pitch". */
   @FXML
   private Label cameraLabel3;
   /** Entry for setting either "x" or "distance". */
   @FXML
   private TextField cameraTextField1;
   /** Entry for setting either "y" or "yaw". */
   @FXML
   private TextField cameraTextField2;
   /** Entry for setting either "z" or "pitch". */
   @FXML
   private TextField cameraTextField3;
   /** Validation for either "x" or "distance". */
   @FXML
   private ImageView cameraValidImageView1;
   /** Validation for either "y" or "yaw". */
   @FXML
   private ImageView cameraValidImageView2;
   /** Validation for either "z" or "pitch". */
   @FXML
   private ImageView cameraValidImageView3;

   @FXML
   private TextField xCameraCurrentTextField, yCameraCurrentTextField, zCameraCurrentTextField;

   private final DoubleSearchField[] cameraCoordinatesSearchFields = new DoubleSearchField[3];
   private final DoubleProperty[] cameraCoordinates = new DoubleProperty[3];

   private final Property<Tuple3DProperty> cameraPositionCoordinatesToTrack = new SimpleObjectProperty<>(this, "cameraPositionCoordinatesToTrack", null);
   private final Property<CameraOrbitalCoordinateProperty> cameraOrbitalCoordinatesToTrack = new SimpleObjectProperty<>(this,
                                                                                                                        "cameraOrbitCoordinatesToTrack",
                                                                                                                        null);
   private final Property<CameraLevelOrbitalCoordinateProperty> cameraLevelOrbitalCoordinatesToTrack = new SimpleObjectProperty<>(this,
                                                                                                                                  "cameraOrbit2DCoordinatesToTrack",
                                                                                                                                  null);

   public void initialize(PerspectiveCameraController cameraController,
                          YoCompositeSearchManager searchManager,
                          LinkedYoRegistry linkedRootRegistry,
                          ReferenceFrameManager referenceFrameManager)
   {
      ToggleGroup toggleGroup = new ToggleGroup();
      toggleGroup.getToggles().addAll(trackCoordinatesButton, trackNodeButton); // TODO initialize which one is selected

      CameraFocalPointHandler focalPointHandler = cameraController.getFocalPointHandler();
      ObjectProperty<TrackingTargetType> targetTypeProperty = focalPointHandler.targetTypeProperty();

      trackNodeButton.setSelected(targetTypeProperty.get() == TrackingTargetType.Node);
      trackCoordinatesButton.setSelected(targetTypeProperty.get() == TrackingTargetType.YoCoordinates);

      MutableBoolean updatingTarget = new MutableBoolean(false);

      toggleGroup.selectedToggleProperty().addListener((o, oldValue, newValue) ->
      {
         if (updatingTarget.booleanValue())
            return;
         updatingTarget.setTrue();

         if (newValue == trackNodeButton)
            targetTypeProperty.set(TrackingTargetType.Node);
         else if (newValue == trackCoordinatesButton)
            targetTypeProperty.set(TrackingTargetType.YoCoordinates);
         else
            targetTypeProperty.set(TrackingTargetType.Disabled);

         updatingTarget.setFalse();
      });

      ChangeListener<? super TrackingTargetType> targetTypeChangeListener = (o, oldValue, newValue) ->
      {
         if (updatingTarget.booleanValue())
            return;
         updatingTarget.setTrue();

         trackNodeButton.setSelected(targetTypeProperty.get() == TrackingTargetType.Node);
         trackCoordinatesButton.setSelected(targetTypeProperty.get() == TrackingTargetType.YoCoordinates);

         updatingTarget.setFalse();
      };
      targetTypeProperty.addListener(targetTypeChangeListener);
      targetTypeChangeListener.changed(targetTypeProperty, null, targetTypeProperty.get());

      ChangeListener<? super Transform> focalPointTransformChangeListener = (o, oldValue, newValue) ->
      {
         xFocalPointCurrentTextField.setText(Double.toString(newValue.getTx()));
         yFocalPointCurrentTextField.setText(Double.toString(newValue.getTy()));
         zFocalPointCurrentTextField.setText(Double.toString(newValue.getTz()));
      };
      Node focalPoint = cameraController.getFocalPointViz();
      focalPoint.localToSceneTransformProperty().addListener(focalPointTransformChangeListener);
      focalPointTransformChangeListener.changed(focalPoint.localToSceneTransformProperty(), null, focalPoint.getLocalToSceneTransform());

      ChangeListener<? super Transform> cameraTransformChangeListener = (o, oldValue, newValue) ->
      {
         xCameraCurrentTextField.setText(Double.toString(newValue.getTx()));
         yCameraCurrentTextField.setText(Double.toString(newValue.getTy()));
         zCameraCurrentTextField.setText(Double.toString(newValue.getTz()));
      };
      PerspectiveCamera camera = cameraController.getCamera();
      camera.localToSceneTransformProperty().addListener(cameraTransformChangeListener);
      cameraTransformChangeListener.changed(camera.localToSceneTransformProperty(), null, camera.getLocalToSceneTransform());

      YoCompositeCollection yoTuple3DCollection = searchManager.getYoTuple3DCollection();
      if (yoTuple3DCollection == null)
      {
         yoCoordinateEditorController.getMainPane().setDisable(true);
         // Happens when no session is loaded
      }
      else
      {
         ObjectProperty<Tuple3DProperty> coordinatesToTrack = focalPointHandler.coordinatesToTrackProperty();
         yoCoordinateEditorController.initialize(searchManager, referenceFrameManager, linkedRootRegistry, yoTuple3DCollection, true);
         yoCoordinateEditorController.setCompositeName("Tracking Coordinates");
         yoCoordinateEditorController.getMainPane().disableProperty().bind(trackCoordinatesButton.selectedProperty().not());
         if (coordinatesToTrack.get() != null)
            yoCoordinateEditorController.setInput(coordinatesToTrack.get());

         yoCoordinateEditorController.addInputListener((coords, frame) -> coordinatesToTrack.set(new Tuple3DProperty(frame, coords)));
      }

      trackingNodeTextField.setEditable(false); // We use this field to provide info about the currently selected Node
      trackingNodeTextField.setTooltip(new Tooltip("To select a node to track, right click on it in the 3D view and select in the context menu."));
      trackingNodeTextField.disableProperty().bind(trackNodeButton.selectedProperty().not());

      ObjectProperty<Node> nodeToTrack = focalPointHandler.nodeToTrackProperty();
      trackingNodeTextField.setText(nodeToTrack.get() == null ? "null" : nodeToTrack.get().getId());

      ChangeListener<? super Node> nodeTrackedChangeListener = (o, oldValue, newValue) ->
      {
         if (newValue == null)
            trackingNodeTextField.setText("No node being tracked");
         else if (newValue.getId() == null || newValue.getId().isBlank())
            trackingNodeTextField.setText("Tracking node w/o id");
         else
            trackingNodeTextField.setText(newValue.getId());
      };
      nodeToTrack.addListener(nodeTrackedChangeListener);
      nodeTrackedChangeListener.changed(nodeToTrack, null, nodeToTrack.get());

      cameraPositionComboxBox.setItems(FXCollections.observableArrayList(CameraPositionType.values()));

      cameraCoordinatesSearchFields[0] = new DoubleSearchField(cameraTextField1, searchManager, linkedRootRegistry, cameraValidImageView1);
      cameraCoordinatesSearchFields[1] = new DoubleSearchField(cameraTextField2, searchManager, linkedRootRegistry, cameraValidImageView2);
      cameraCoordinatesSearchFields[2] = new DoubleSearchField(cameraTextField3, searchManager, linkedRootRegistry, cameraValidImageView3);

      for (int i = 0; i < cameraCoordinatesSearchFields.length; i++)
      {
         cameraCoordinatesSearchFields[i].setupAutoCompletion();

         int supplierIndex = i;
         cameraCoordinatesSearchFields[i].supplierProperty().addListener((o, oldValue, newValue) ->
         {
            cameraCoordinates[supplierIndex] = newValue;
            switch (cameraPositionComboxBox.getSelectionModel().getSelectedItem())
            {
               case Position:
                  cameraPositionCoordinatesToTrack.getValue().setComponentValueProperties(cameraCoordinates);
                  break;
               case Orbital:
                  cameraOrbitalCoordinatesToTrack.getValue().setComponentValueProperties(cameraCoordinates);
                  break;
               case LevelOrbital:
                  cameraLevelOrbitalCoordinatesToTrack.getValue().setComponentValueProperties(cameraCoordinates);
                  break;
            }
         });
      }

      cameraPositionComboxBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
      {
         switch (newValue)
         {
            case Position:
               cameraLabel1.setText("x");
               cameraLabel2.setText("y");
               cameraLabel3.setText("z");
               break;
            case Orbital:
               cameraLabel1.setText("distance");
               cameraLabel2.setText("longitude");
               cameraLabel3.setText("latitude");
               break;
            case LevelOrbital:
               cameraLabel1.setText("distance");
               cameraLabel2.setText("longitude");
               cameraLabel3.setText("z");
               break;
            default:
               throw new IllegalStateException("Unexpected type: " + newValue);
         }
      });

      Transform currentCameraPose = cameraController.getCamera().getLocalToSceneTransform();
      CameraOrbitHandler orbitHandler = cameraController.getOrbitHandler();

      if (cameraController.cameraPositionCoordinatesToTrackProperty().getValue() != null)
      {
         cameraPositionCoordinatesToTrack.setValue(new Tuple3DProperty(cameraController.cameraPositionCoordinatesToTrackProperty().getValue()));
         cameraPositionComboxBox.getSelectionModel().select(CameraPositionType.Position);
      }
      else
      {
         cameraPositionCoordinatesToTrack.setValue(new Tuple3DProperty(currentCameraPose.getTx(), currentCameraPose.getTy(), currentCameraPose.getTz()));
      }

      if (cameraController.cameraOrbitalCoordinatesToTrackProperty().getValue() != null)
      {
         cameraOrbitalCoordinatesToTrack.setValue(new CameraOrbitalCoordinateProperty(cameraController.cameraOrbitalCoordinatesToTrackProperty().getValue()));
         cameraPositionComboxBox.getSelectionModel().select(CameraPositionType.Orbital);
      }
      else
      {
         cameraOrbitalCoordinatesToTrack.setValue(new CameraOrbitalCoordinateProperty(orbitHandler.distanceProperty().get(),
                                                                                      orbitHandler.longitudeProperty().get(),
                                                                                      orbitHandler.latitudeProperty().get()));
      }

      if (cameraController.cameraLevelOrbitalCoordinatesToTrackProperty().getValue() != null)
      {
         cameraLevelOrbitalCoordinatesToTrack.setValue(new CameraLevelOrbitalCoordinateProperty(cameraController.cameraLevelOrbitalCoordinatesToTrackProperty()
                                                                                                                .getValue()));
         cameraPositionComboxBox.getSelectionModel().select(CameraPositionType.LevelOrbital);
      }
      else
      {
         cameraLevelOrbitalCoordinatesToTrack.setValue(new CameraLevelOrbitalCoordinateProperty(orbitHandler.distanceProperty().get(),
                                                                                                orbitHandler.longitudeProperty().get(),
                                                                                                currentCameraPose.getTz()));
      }

      cameraPositionCoordinatesToTrack.addListener((o, oldValue, newValue) -> cameraController.cameraPositionCoordinatesToTrackProperty().setValue(newValue));
      cameraLevelOrbitalCoordinatesToTrack.addListener((o, oldValue, newValue) -> cameraController.cameraLevelOrbitalCoordinatesToTrackProperty()
                                                                                                  .setValue(newValue));
      cameraLevelOrbitalCoordinatesToTrack.addListener((o, oldValue, newValue) -> cameraController.cameraLevelOrbitalCoordinatesToTrackProperty()
                                                                                                  .setValue(newValue));

   }
}
