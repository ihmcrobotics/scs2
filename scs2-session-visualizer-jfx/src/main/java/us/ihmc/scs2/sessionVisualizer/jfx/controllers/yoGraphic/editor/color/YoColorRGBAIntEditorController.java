package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.color;

import java.util.Arrays;

import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBAIntDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.YoColorRGBAIntFX;

public class YoColorRGBAIntEditorController extends PaintEditorController<YoColorRGBAIntFX>
{
   @FXML
   private GridPane mainPane;
   @FXML
   private Label searchRedLabel, searchGreenLabel, searchBlueLabel, searchAlphaLabel;
   @FXML
   private TextField searchRedTextField, searchGreenTextField, searchBlueTextField, searchAlphaTextField;
   @FXML
   private ImageView redValidImageView, greenValidImageView, blueValidImageView, alphaValidImageView;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit)
   {
      super.initialize(toolkit, new YoColorRGBAIntFX());

      for (Label label : Arrays.asList(searchRedLabel, searchGreenLabel, searchBlueLabel, searchAlphaLabel))
         label.setText(label.getText() + " [0-255]");

      setupIntegerPropertyEditor(searchRedTextField, redValidImageView, YoColorRGBAIntFX::setRed);
      setupIntegerPropertyEditor(searchGreenTextField, greenValidImageView, YoColorRGBAIntFX::setGreen);
      setupIntegerPropertyEditor(searchBlueTextField, blueValidImageView, YoColorRGBAIntFX::setBlue);
      setupIntegerPropertyEditor(searchAlphaTextField, alphaValidImageView, YoColorRGBAIntFX::setAlpha);
   }

   @Override
   public void setInput(PaintDefinition input)
   {
      if (input instanceof YoColorRGBAIntDefinition colorDefinition)
         setInput(colorDefinition);
      else
         LogTools.error("Unexpected input: {}", input);
   }

   public void setInput(YoColorRGBAIntDefinition definition)
   {
      String red = definition.getRed();
      String green = definition.getGreen();
      String blue = definition.getBlue();
      String alpha = definition.getAlpha();
      if (alpha == null)
         alpha = Double.toString(1.0);

      searchRedTextField.setText(red);
      searchGreenTextField.setText(green);
      searchBlueTextField.setText(blue);
      searchAlphaTextField.setText(alpha);
   }

   @Override
   public Pane getMainPane()
   {
      return mainPane;
   }
}
