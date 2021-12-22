package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager;

public class YoSliderboardMenuController
{
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
   }

   @FXML
   public void openBCF2000SliderboardWindow()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), new Pair<>(SecondaryWindowManager.BCF2000_SLIDERBOARD_WINDOW_TYPE, null));
   }
}
