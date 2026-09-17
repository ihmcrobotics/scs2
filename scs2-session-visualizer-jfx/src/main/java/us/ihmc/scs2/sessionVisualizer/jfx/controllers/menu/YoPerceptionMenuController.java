package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager;

public class YoPerceptionMenuController implements VisualizerController
{
   @FXML
   private CheckMenuItem showHeightMapMenuItem;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      SCS2Messager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      messager.bindBidirectional(topics.getShowHeightMap(), showHeightMapMenuItem.selectedProperty(), false);
   }
}
