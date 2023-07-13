package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.control.Tooltip;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.camera.PerspectiveCameraController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.DoubleSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.ReferenceFrameSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

public class Camera3DOptionsPaneController
{
   private enum Tracking
   {
      Frame, Node
   };

   @FXML
   private TextField focalPointXTextField, focalPointYTextField, focalPointZTextField;
   @FXML
   private ImageView focalPointXValidImageView, focalPointYValidImageView, focalPointZValidImageView, focalPointTrackingValidImageView;
   @FXML
   private ComboBox<Tracking> trackingComboBox;
   @FXML
   private TextField trackingFrameTextField, trackingNodeTextField; // Depending on the comboBox state, we flip which TextField is visible

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

   private DoubleSearchField[] yoFocalPointFields = new DoubleSearchField[3];
   private ReferenceFrameSearchField trackingFrameField;

   public Camera3DOptionsPaneController()
   {
   }

   public void initialize(PerspectiveCameraController cameraController,
                          YoCompositeSearchManager searchManager,
                          LinkedYoRegistry linkedRootRegistry,
                          ReferenceFrameManager referenceFrameManager)
   {
      yoFocalPointFields[0] = new DoubleSearchField(focalPointXTextField, searchManager, linkedRootRegistry, focalPointXValidImageView);
      yoFocalPointFields[1] = new DoubleSearchField(focalPointYTextField, searchManager, linkedRootRegistry, focalPointYValidImageView);
      yoFocalPointFields[2] = new DoubleSearchField(focalPointZTextField, searchManager, linkedRootRegistry, focalPointZValidImageView);
      trackingComboBox.setItems(FXCollections.observableArrayList(Tracking.values()));

      trackingFrameField = new ReferenceFrameSearchField(trackingFrameTextField, referenceFrameManager, focalPointTrackingValidImageView);
      trackingNodeTextField.setEditable(false); // We use this field to provide info about the currently selected Node
      trackingNodeTextField.setTooltip(new Tooltip("To select a node to track, right click on it in the 3D view and select in the context menu."));

      trackingComboBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
      {
         switch (newValue)
         {
            case Frame:
               trackingFrameTextField.setVisible(true);
               focalPointTrackingValidImageView.setVisible(true);
               trackingNodeTextField.setVisible(false);
               break;
            case Node:
               trackingFrameTextField.setVisible(false);
               focalPointTrackingValidImageView.setVisible(false);
               trackingNodeTextField.setVisible(true);
               break;
            default:
               throw new IllegalStateException("Unexpected value: " + newValue);
         }
      });

      trackingComboBox.getSelectionModel().select(Tracking.Node);

      cameraPositionComboxBox.setItems(FXCollections.observableArrayList(CameraPositionType.values()));
   }
}
