package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.stage.Stage;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoSliderboardMenuController implements VisualizerController
{
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private Stage owner;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      owner = toolkit.getWindow();
   }

   @FXML
   public void openBCF2000SliderboardWindow()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), NewWindowRequest.bfc2000SliderboardWindow(owner));
   }

   @FXML
   public void openXTouchSliderboardWindow()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), NewWindowRequest.xtouchCompactSliderboardWindow(owner));
   }
}
