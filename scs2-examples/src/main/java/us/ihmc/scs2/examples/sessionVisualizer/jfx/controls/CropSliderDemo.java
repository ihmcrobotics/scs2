package us.ihmc.scs2.examples.sessionVisualizer.jfx.controls;

import com.jfoenix.controls.JFXSlider.IndicatorPosition;

import javafx.application.Application;
import javafx.geometry.Insets;
import javafx.geometry.Orientation;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CropSlider;

public class CropSliderDemo extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      ToggleButton showTrimButton = new ToggleButton("Show trim");
      CropSlider cropSlider1 = new CropSlider();
      cropSlider1.showTrimProperty().bind(showTrimButton.selectedProperty());
      cropSlider1.setOrientation(Orientation.HORIZONTAL);
      cropSlider1.setIndicatorPosition(IndicatorPosition.LEFT);

      CropSlider cropSlider2 = new CropSlider();
      cropSlider2.showTrimProperty().bind(showTrimButton.selectedProperty());
      cropSlider2.setOrientation(Orientation.HORIZONTAL);
      cropSlider2.setIndicatorPosition(IndicatorPosition.RIGHT);

      CropSlider cropSlider3 = new CropSlider();
      cropSlider3.showTrimProperty().bind(showTrimButton.selectedProperty());
      cropSlider3.setOrientation(Orientation.VERTICAL);
      cropSlider3.setIndicatorPosition(IndicatorPosition.LEFT);

      CropSlider cropSlider4 = new CropSlider();
      cropSlider4.showTrimProperty().bind(showTrimButton.selectedProperty());
      cropSlider4.setOrientation(Orientation.VERTICAL);
      cropSlider4.setIndicatorPosition(IndicatorPosition.RIGHT);
      
      VBox pane = new VBox(60.0, showTrimButton, cropSlider1, cropSlider2, cropSlider3, cropSlider4);
      pane.alignmentProperty().set(Pos.CENTER);
      pane.setPadding(new Insets(50.0));
      Scene scene = new Scene(pane);
      primaryStage.setScene(scene);
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
