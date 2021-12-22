package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import java.io.File;

import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.stage.Stage;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoGraphicMenuController
{
   @FXML
   private CheckMenuItem overheadPlotterMenuItem;

   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private Stage owner;

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      owner = toolkit.getWindow();

      messager.bindBidirectional(topics.getShowOverheadPlotter(), overheadPlotterMenuItem.selectedProperty(), false);
   }

   @FXML
   private void loadYoGraphic()
   {
      File result = SessionVisualizerIOTools.yoGraphicConfigurationOpenFileDialog(owner);
      if (result != null)
         messager.submitMessage(topics.getYoGraphicLoadRequest(), result);
   }

   @FXML
   private void saveYoGraphic()
   {
      File result = SessionVisualizerIOTools.yoGraphicConfigurationSaveFileDialog(owner);
      if (result != null)
         messager.submitMessage(topics.getYoGraphicSaveRequest(), result);
   }

   @FXML
   private void openYoGraphicEditor()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), new Pair<>(SecondaryWindowManager.GRAPHIC_EDITOR_WINDOW_TYPE, null));
   }
}
