package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import java.io.File;

import javafx.fxml.FXML;
import javafx.stage.Stage;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoCompositeMenuController
{
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private Stage mainWindow;

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      mainWindow = toolkit.getWindow();
   }

   @FXML
   public void openYoCompositePatternEditor()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), new Pair<>(SecondaryWindowManager.COMPOSITE_PATTERN_EDITOR_WINDOW_TYPE, null));
   }

   @FXML
   public void loadYoCompositePattern()
   {
      File result = SessionVisualizerIOTools.yoCompositeConfigurationOpenFileDialog(mainWindow);
      if (result != null)
         messager.submitMessage(topics.getYoCompositePatternLoadRequest(), result);
   }

   @FXML
   public void saveYoCompositePattern()
   {
      File result = SessionVisualizerIOTools.yoCompositeConfigurationSaveFileDialog(mainWindow);
      if (result != null)
         messager.submitMessage(topics.getYoCompositePatternSaveRequest(), result);
   }

   @FXML
   public void refreshAllYoComposite()
   {
      messager.submitMessage(topics.getYoCompositeRefreshAll(), true);
   }
}
