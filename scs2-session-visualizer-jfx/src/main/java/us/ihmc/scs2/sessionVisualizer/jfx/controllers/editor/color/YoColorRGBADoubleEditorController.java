package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.color;

import java.util.Arrays;

import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBADoubleDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.YoColorRGBADoubleFX;

public class YoColorRGBADoubleEditorController extends PaintEditorController<YoColorRGBADoubleFX>
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
      super.initialize(toolkit, new YoColorRGBADoubleFX());

      for (Label label : Arrays.asList(searchRedLabel, searchGreenLabel, searchBlueLabel, searchAlphaLabel))
         label.setText(label.getText() + " [0.0-1.0]");

      setupDoublePropertyEditor(searchRedTextField, redValidImageView, YoColorRGBADoubleFX::setRed);
      setupDoublePropertyEditor(searchGreenTextField, greenValidImageView, YoColorRGBADoubleFX::setGreen);
      setupDoublePropertyEditor(searchBlueTextField, blueValidImageView, YoColorRGBADoubleFX::setBlue);
      setupDoublePropertyEditor(searchAlphaTextField, alphaValidImageView, YoColorRGBADoubleFX::setAlpha);
   }

   @Override
   public void setInput(PaintDefinition input)
   {
      if (input instanceof YoColorRGBADoubleDefinition colorDefinition)
         setInput(colorDefinition);
      else
         LogTools.error("Unexpected input: {}", input);
   }

   public void setInput(YoColorRGBADoubleDefinition definition)
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
