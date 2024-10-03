package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import javafx.fxml.FXML;
import javafx.scene.control.MenuItem;
import javafx.stage.Window;
import javafx.util.Pair;
import org.kordamp.ikonli.javafx.FontIcon;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

import java.io.File;

public class YoChartMenuController implements VisualizerController
{
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private Window owner;

   private boolean isYAxisVisible = false;

   @FXML
   private MenuItem toggleYAxisMenuItem;

   @FXML
   private FontIcon toggleYAxisIcon;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      owner = toolkit.getWindow();
   }

   @FXML
   public void toggleChartYAxis()
   {
      isYAxisVisible = !isYAxisVisible;
      messager.submitMessage(topics.getYoChartShowYAxis(), new Pair<>(owner, isYAxisVisible));
      updateMenuItemText();
   }

   private void updateMenuItemText()
   {
      if (isYAxisVisible)
      {
         toggleYAxisMenuItem.setText("Hide Charts Y-Axis");
         toggleYAxisIcon.getStyleClass().clear();
         toggleYAxisIcon.getStyleClass().addAll("hide-icon-view", "menu-item-icon-view");
      }
      else
      {
         toggleYAxisMenuItem.setText("Show Charts Y-Axis");
         toggleYAxisIcon.getStyleClass().clear();
         toggleYAxisIcon.getStyleClass().addAll("show-icon-view", "menu-item-icon-view");
      }
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
         messager.submitMessage(topics.getYoChartGroupSaveConfiguration(), new Pair<>(owner, result));
   }

   @FXML
   public void newChartWindow()
   {
      messager.submitMessage(topics.getOpenWindowRequest(), NewWindowRequest.chartWindow(owner));
   }
}
