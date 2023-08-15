package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.color;

import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.YoColorRGBASingleFX;

public class YoColorRGBASingleEditorController extends PaintEditorController<YoColorRGBASingleFX>
{
   @FXML
   private Pane mainPane;
   @FXML
   private Label searchRGBALabel;
   @FXML
   private TextField searchRGBATextField;
   @FXML
   private ImageView rgbaValidImageView;

   @Override
   public void initialize(SessionVisualizerToolkit toolkit)
   {
      super.initialize(toolkit, new YoColorRGBASingleFX());
      setupIntegerPropertyEditor(searchRGBATextField, rgbaValidImageView, YoColorRGBASingleFX::setRGBA);
   }

   @Override
   public void setInput(PaintDefinition input)
   {
      if (input instanceof YoColorRGBASingleDefinition colorDefinition)
         setInput(colorDefinition);
      else
         LogTools.error("Unexpected input: {}", input);
   }

   public void setInput(YoColorRGBASingleDefinition definition)
   {
      searchRGBATextField.setText(definition.getRGBA());
   }

   @Override
   public Pane getMainPane()
   {
      return mainPane;
   }
}
