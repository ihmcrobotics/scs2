package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SecondaryWindowControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.YoChartGroupPanelController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu.MainWindowMenuBarController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager;

public class SecondaryWindowController
{
   @FXML
   private VBox mainNode;
   @FXML
   private MainWindowMenuBarController menuController;
   @FXML
   private SecondaryWindowControlsController controlsController;

   private SessionVisualizerWindowToolkit toolkit;

   public void initialize(SessionVisualizerToolkit globalToolkit, Stage owner)
   {
      toolkit = new SessionVisualizerWindowToolkit(owner, globalToolkit);
      menuController.initialize(toolkit);
      controlsController.initialize(toolkit);

      owner.addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (e.isConsumed())
            return;
         closeAndDispose();
         owner.close();
      });
   }

   private YoChartGroupPanelController chartGroupController = null;

   public void setupChartGroup() throws IOException
   {
      Stage stage = toolkit.getWindow();

      // Loading the chart pane & controller
      FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.CHART_GROUP_PANEL_URL);
      AnchorPane chartGroupPane = loader.load();
      chartGroupController = loader.getController();

      chartGroupController.initialize(toolkit);
      chartGroupController.start();
      chartGroupController.maximumRowProperty().set(9);
      chartGroupController.maximumColProperty().set(6);

      SessionVisualizerIOTools.addSCSIconToWindow(stage);

      mainNode.getChildren().set(1, chartGroupPane);
      VBox.setVgrow(chartGroupPane, Priority.ALWAYS);
      Scene scene = new Scene(mainNode, 1024, 768);
      stage.setScene(scene);
      String windowTitlePrefix = "Chart Window";
      stage.setTitle(windowTitlePrefix);
      chartGroupController.chartGroupNameProperty().addListener((o, oldValue, newValue) ->
      {
         if (newValue == null || newValue.isEmpty())
            stage.setTitle(windowTitlePrefix);
         else
            stage.setTitle(windowTitlePrefix + ": " + newValue);
      });
      stage.addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (e.isConsumed())
            return;
         stage.close();
         chartGroupController.closeAndDispose();
      });
   }

   public void saveSessionConfiguration(SCSGuiConfiguration configuration)
   {
      configuration.addSecondaryWindowConfiguration(SecondaryWindowManager.toWindowConfigurationDefinition(toolkit.getWindow()));
      chartGroupController.saveChartGroupConfiguration(toolkit.getWindow(), configuration.addSecondaryYoChartGroupConfigurationFile());
   }

   public void start()
   {
      toolkit.start();
   }

   public void closeAndDispose()
   {
      chartGroupController.closeAndDispose();
      toolkit.getWindow().close();
      toolkit.stop();
   }
}
