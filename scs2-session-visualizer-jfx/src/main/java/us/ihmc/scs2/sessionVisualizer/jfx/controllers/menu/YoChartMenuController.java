package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import java.io.File;

import javafx.fxml.FXML;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoChartMenuController implements VisualizerController
{
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private Window owner;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      owner = toolkit.getWindow();
   }

   @FXML
   public void loadChartGroup()
   {
      File result = SessionVisualizerIOTools.yoChartConfigurationOpenFileDialog(owner);
      if (result != null)
         messager.submitMessage(topics.getYoChartGroupLoadConfiguration(), new Pair<>(owner, result));
   }

   @FXML
   public void saveChartGroup()
   {
      File result = SessionVisualizerIOTools.yoChartConfigurationSaveFileDialog(owner);
      if (result != null)
      {
         messager.submitMessage(topics.getYoChartGroupName(),
                                new Pair<>(owner, result.getName().replace(SessionVisualizerIOTools.yoChartGroupConfigurationFileExtension, "")));
         messager.submitMessage(topics.getYoChartGroupSaveConfiguration(), new Pair<>(owner, result));
      }
   }

   @FXML
   public void newChartWindow()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), NewWindowRequest.chartWindow(owner));
   }
}
