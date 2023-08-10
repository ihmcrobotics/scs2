package us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyObjectProperty;
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
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.CompositeProperty;
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

   @FXML
   private ComboBox<CameraControlMode> cameraPositionComboxBox;

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

   private Label[] cameraLabels;
   private TextField[] cameraTextFields;
   private final DoubleSearchField[] cameraCoordinatesSearchFields = new DoubleSearchField[3];

   private final List<Runnable> cleanupTasks = new ArrayList<>();

   public void initialize(PerspectiveCameraController cameraController,
                          YoCompositeSearchManager searchManager,
                          LinkedYoRegistry linkedRootRegistry,
                          ReferenceFrameManager referenceFrameManager)
   {
      initializeFocalPointControls(cameraController, searchManager, linkedRootRegistry, referenceFrameManager);
      initializeCameraControls(cameraController, searchManager, linkedRootRegistry);
   }

   public void initializeFocalPointControls(PerspectiveCameraController cameraController,
                                            YoCompositeSearchManager searchManager,
                                            LinkedYoRegistry linkedRootRegistry,
                                            ReferenceFrameManager referenceFrameManager)
   {
      PerspectiveCamera camera = cameraController.getCamera();
      CameraFocalPointHandler focalPointHandler = cameraController.getFocalPointHandler();
      ObjectProperty<TrackingTargetType> targetTypeProperty = focalPointHandler.targetTypeProperty();

      ToggleGroup toggleGroup = new ToggleGroup();
      toggleGroup.getToggles().addAll(trackCoordinatesButton, trackNodeButton); // TODO initialize which one is selected

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
      cleanupTasks.add(() -> targetTypeProperty.removeListener(targetTypeChangeListener));
      targetTypeChangeListener.changed(targetTypeProperty, null, targetTypeProperty.get());

      ChangeListener<? super Transform> focalPointTransformChangeListener = (o, oldValue, newValue) ->
      {
         xFocalPointCurrentTextField.setText(Double.toString(newValue.getTx()));
         yFocalPointCurrentTextField.setText(Double.toString(newValue.getTy()));
         zFocalPointCurrentTextField.setText(Double.toString(newValue.getTz()));
      };
      Node focalPoint = cameraController.getFocalPointViz();
      focalPoint.localToSceneTransformProperty().addListener(focalPointTransformChangeListener);
      cleanupTasks.add(() -> focalPoint.localToSceneTransformProperty().removeListener(focalPointTransformChangeListener));
      focalPointTransformChangeListener.changed(focalPoint.localToSceneTransformProperty(), null, focalPoint.getLocalToSceneTransform());

      ChangeListener<? super Transform> cameraTransformChangeListener = (o, oldValue, newValue) ->
      {
         xCameraCurrentTextField.setText(Double.toString(newValue.getTx()));
         yCameraCurrentTextField.setText(Double.toString(newValue.getTy()));
         zCameraCurrentTextField.setText(Double.toString(newValue.getTz()));
      };
      camera.localToSceneTransformProperty().addListener(cameraTransformChangeListener);
      cleanupTasks.add(() -> camera.localToSceneTransformProperty().removeListener(cameraTransformChangeListener));
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
      cleanupTasks.add(() -> nodeToTrack.removeListener(nodeTrackedChangeListener));
      nodeTrackedChangeListener.changed(nodeToTrack, null, nodeToTrack.get());
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   public void initializeCameraControls(PerspectiveCameraController cameraController,
                                        YoCompositeSearchManager searchManager,
                                        LinkedYoRegistry linkedRootRegistry)
   {
      ReadOnlyObjectProperty<Transform> currentCameraPose = cameraController.getCamera().localToSceneTransformProperty();
      CameraOrbitHandler orbitHandler = cameraController.getOrbitHandler();
      Property<Tuple3DProperty> cameraPositionCoordinatesToTrack = cameraController.cameraPositionCoordinatesToTrackProperty();
      Property<OrbitalCoordinateProperty> cameraOrbitalCoordinatesToTrack = cameraController.cameraOrbitalCoordinatesToTrackProperty();
      Property<LevelOrbitalCoordinateProperty> cameraLevelOrbitalCoordinatesToTrack = cameraController.cameraLevelOrbitalCoordinatesToTrackProperty();

      cameraLabels = new Label[] {cameraLabel1, cameraLabel2, cameraLabel3};
      cameraTextFields = new TextField[] {cameraTextField1, cameraTextField2, cameraTextField3};

      cameraPositionComboxBox.setItems(FXCollections.observableArrayList(CameraControlMode.values()));
      cameraCoordinatesSearchFields[0] = new DoubleSearchField(cameraTextField1, searchManager, linkedRootRegistry, cameraValidImageView1);
      cameraCoordinatesSearchFields[1] = new DoubleSearchField(cameraTextField2, searchManager, linkedRootRegistry, cameraValidImageView2);
      cameraCoordinatesSearchFields[2] = new DoubleSearchField(cameraTextField3, searchManager, linkedRootRegistry, cameraValidImageView3);

      MutableBoolean updatingCameraTextFields = new MutableBoolean(false);

      for (int i = 0; i < cameraCoordinatesSearchFields.length; i++)
      {
         cameraCoordinatesSearchFields[i].setupAutoCompletion();

         int supplierIndex = i;
         cameraCoordinatesSearchFields[i].supplierProperty().addListener((o, oldValue, newValue) ->
         {
            if (updatingCameraTextFields.booleanValue())
               return;

            CameraControlMode controlMode = cameraController.cameraControlMode().getValue();
            CompositeProperty composite = switch (controlMode)
            {
               case Position -> cameraPositionCoordinatesToTrack.getValue();
               case Orbital -> cameraOrbitalCoordinatesToTrack.getValue();
               case LevelOrbital -> cameraLevelOrbitalCoordinatesToTrack.getValue();
            };
            DoubleProperty[] components = Arrays.copyOf(composite.componentValueProperties(), 3);
            components[supplierIndex] = newValue;

            switch (controlMode)
            {
               case Position:
                  cameraPositionCoordinatesToTrack.setValue(new Tuple3DProperty(components));
                  break;
               case Orbital:
                  cameraOrbitalCoordinatesToTrack.setValue(new OrbitalCoordinateProperty(components));
                  break;
               case LevelOrbital:
                  cameraLevelOrbitalCoordinatesToTrack.setValue(new LevelOrbitalCoordinateProperty(components));
                  break;
            }
         });
      }

      cameraPositionComboxBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
      {
         cameraController.cameraControlMode().setValue(newValue);

         CompositeProperty composite = switch (newValue)
         {
            case Position:
               if (cameraPositionCoordinatesToTrack.getValue() == null)
                  cameraPositionCoordinatesToTrack.setValue(new Tuple3DProperty(currentCameraPose.get().getTx(),
                                                                                currentCameraPose.get().getTy(),
                                                                                currentCameraPose.get().getTz()));
               yield cameraPositionCoordinatesToTrack.getValue();
            case Orbital:
               if (cameraOrbitalCoordinatesToTrack.getValue() == null)
                  cameraOrbitalCoordinatesToTrack.setValue(new OrbitalCoordinateProperty(orbitHandler.distanceProperty().get(),
                                                                                         orbitHandler.longitudeProperty().get(),
                                                                                         orbitHandler.latitudeProperty().get()));
               yield cameraOrbitalCoordinatesToTrack.getValue();
            case LevelOrbital:
               if (cameraLevelOrbitalCoordinatesToTrack.getValue() == null)
                  cameraLevelOrbitalCoordinatesToTrack.setValue(new LevelOrbitalCoordinateProperty(orbitHandler.distanceProperty().get(),
                                                                                                   orbitHandler.longitudeProperty().get(),
                                                                                                   currentCameraPose.get().getTz()));
               yield cameraLevelOrbitalCoordinatesToTrack.getValue();
            default:
               throw new IllegalStateException("Unexpected type: " + newValue);
         };

         updatingCameraTextFields.setTrue();
         for (int i = 0; i < 3; i++)
         {
            cameraLabels[i].setText(composite.getComponentIdentifiers()[i]);
            cameraTextFields[i].setText(CompositePropertyTools.toDoublePropertyName(composite.componentValueProperties()[i]));
         }
         updatingCameraTextFields.setFalse();
      });

      {
         CameraControlMode[] controlModes = {CameraControlMode.Position, CameraControlMode.Orbital, CameraControlMode.LevelOrbital};
         Property[] cameraCoordinatesProperties = {cameraPositionCoordinatesToTrack, cameraOrbitalCoordinatesToTrack, cameraLevelOrbitalCoordinatesToTrack};

         for (int i = 0; i < cameraCoordinatesProperties.length; i++)
         {
            CameraControlMode controlMode = controlModes[i];
            Property cameraCoordinatesProperty = cameraCoordinatesProperties[i];

            List<ChangeListener<? super Number>> componentChangeListeners = new ArrayList<>();
            for (int componentIndex = 0; componentIndex < 3; componentIndex++)
            {
               int componentIndexFinal = componentIndex;
               componentChangeListeners.add((o, oldValue, newValue) ->
               {
                  if (cameraController.cameraControlMode().getValue() != controlMode)
                     return;
                  if (cameraTextFields[componentIndexFinal].isFocused())
                     return;
                  updatingCameraTextFields.setTrue();
                  CompositeProperty cameraCoordinates = (CompositeProperty) cameraCoordinatesProperty.getValue();
                  cameraTextFields[componentIndexFinal].setText(CompositePropertyTools.toDoublePropertyName(cameraCoordinates.componentValueProperties()[componentIndexFinal]));
                  updatingCameraTextFields.setFalse();
               });
            }

            ChangeListener<? super CompositeProperty> cameraCoordinatesChangeListener = (o, oldValue, newValue) ->
            {
               if (oldValue != null && oldValue.componentValueProperties() != null)
               {
                  for (int componentIndex = 0; componentIndex < 3; componentIndex++)
                  {
                     DoubleProperty componentValueProperty = oldValue.componentValueProperties()[componentIndex];
                     if (componentValueProperty != null && !(componentValueProperty instanceof YoDoubleProperty))
                        componentValueProperty.removeListener(componentChangeListeners.get(componentIndex));
                  }
               }

               if (newValue != null && newValue.componentValueProperties() != null)
               {
                  for (int componentIndex = 0; componentIndex < 3; componentIndex++)
                  {
                     DoubleProperty componentValueProperty = newValue.componentValueProperties()[componentIndex];
                     if (componentValueProperty != null && !(componentValueProperty instanceof YoDoubleProperty))
                        componentValueProperty.addListener(componentChangeListeners.get(componentIndex));
                  }
               }
            };
            cameraCoordinatesProperty.addListener(cameraCoordinatesChangeListener);
            cleanupTasks.add(() ->
            {
               cameraCoordinatesProperty.removeListener(cameraCoordinatesChangeListener);
               // Remove component listeners
               cameraCoordinatesChangeListener.changed(null, (CompositeProperty) cameraCoordinatesProperty.getValue(), null);
            });
         }
      }

      cameraPositionComboxBox.getSelectionModel().select(cameraController.cameraControlMode().getValue());
   }

   public void closeAndDispose()
   {
      cleanupTasks.forEach(Runnable::run);
      cleanupTasks.clear();
   }
}
