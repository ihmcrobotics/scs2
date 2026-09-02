package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoPerceptionMenuController implements VisualizerController
{
   @FXML
   private CheckMenuItem showHeightScanMenuItem;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      messager.bindBidirectional(topics.getShowHeightScan(), showHeightScanMenuItem.selectedProperty(), false);
   }
}
