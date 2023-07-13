package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoDoubleTextField;

public class Camera3DOptionsPaneController
{
   private enum Tracking
   {
      Frame, Node
   };

   @FXML
   private TextField focalPointXTextField, focalPointYTextField, focalPointZTextField;
   @FXML
   private ComboBox<Tracking> trackingComboBox;
   @FXML
   private ImageView focalPointXValidImageView, focalPointYValidImageView, focalPointZValidImageView, focalPointTrackingValidImageView;

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

   public Camera3DOptionsPaneController()
   {
   }

   public void initialize()
   {
      new YoDoubleTextField(focalPointXTextField, searchManager, linkedRootRegistry, focalPointXValidImageView);
      trackingComboBox.setItems(FXCollections.observableArrayList(Tracking.values()));
      cameraPositionComboxBox.setItems(FXCollections.observableArrayList(CameraPositionType.values()));
   }
}
