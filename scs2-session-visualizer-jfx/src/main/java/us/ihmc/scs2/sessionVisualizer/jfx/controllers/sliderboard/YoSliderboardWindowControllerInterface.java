package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

import java.io.File;

import javafx.beans.property.StringProperty;
import javafx.fxml.FXML;
import javafx.stage.Window;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;

public interface YoSliderboardWindowControllerInterface
{

   void initialize(Window owner, SessionVisualizerToolkit toolkit);

   void importYoSliderboard();

   void load(File file);

   void exportYoSliderboard();

   void save(File file);

   void setInput(YoSliderboardDefinition input);

   void setButtonInput(YoButtonDefinition buttonDefinition);

   void removeButtonInput(int buttonIndex);

   void setKnobInput(YoKnobDefinition knobDefinition);

   void removeKnobInput(int knobIndex);

   void setSliderInput(YoSliderDefinition sliderDefinition);

   void removeSliderInput(int sliderIndex);

   void clear();

   void start();

   void stop();

   void close();

   StringProperty nameProperty();

   YoSliderboardDefinition toYoSliderboardDefinition();

   boolean isEmpty();

}