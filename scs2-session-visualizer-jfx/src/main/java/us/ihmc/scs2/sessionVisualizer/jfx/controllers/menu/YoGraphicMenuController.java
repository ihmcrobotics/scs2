package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import static us.ihmc.scs2.sessionVisualizer.jfx.managers.NewTerrainVisualRequest.visible;
import static us.ihmc.scs2.sessionVisualizer.jfx.managers.NewTerrainVisualRequest.wireframeMode;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest.ALL_ROBOTS;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest.visible;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest.wireframeMode;

import java.io.File;
import java.io.IOException;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.CheckMenuItem;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.Plotter2DOptionsStageController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;

public class YoGraphicMenuController implements VisualizerController
{
   @FXML
   private CheckMenuItem overheadPlotterMenuItem;
   @FXML
   private CheckMenuItem showRobotMenuItem, enableWireframeRobotModeMenuItem;
   @FXML
   private CheckMenuItem showTerrainMenuItem, enableWireframeTerrainModeMenuItem;

   private SessionVisualizerWindowToolkit toolkit;
   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private Stage owner;

   private Property<Plotter2DOptionsStageController> plotterOptionsController = new SimpleObjectProperty<>(this, "plotterOptionsController", null);

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      owner = toolkit.getWindow();

      messager.bindBidirectional(topics.getShowOverheadPlotter(), overheadPlotterMenuItem.selectedProperty(), false);
      showRobotMenuItem.selectedProperty()
                       .addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getRobotVisualRequest(), visible(ALL_ROBOTS, newValue)));
      enableWireframeRobotModeMenuItem.selectedProperty()
                                      .addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getRobotVisualRequest(),
                                                                                                     wireframeMode(ALL_ROBOTS, newValue)));
      showTerrainMenuItem.selectedProperty()
                         .addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getTerrainVisualRequest(), visible(newValue)));
      enableWireframeTerrainModeMenuItem.selectedProperty()
                                        .addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getTerrainVisualRequest(),
                                                                                                       wireframeMode(newValue)));
   }

   @FXML
   private void openPlotter2DOptions()
   {
      if (plotterOptionsController.getValue() != null)
      {
         plotterOptionsController.getValue().close();
         plotterOptionsController.setValue(null);
      }

      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.PLOTTER2D_OPTIONS_STAGE_URL);
         loader.load();
         Plotter2DOptionsStageController controller = loader.getController();
         controller.initialize(toolkit);
         controller.getStage().show();
         plotterOptionsController.setValue(controller);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
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
      messager.submitMessage(topics.getOpenWindowRequest(), NewWindowRequest.graphicEditorWindow(owner));
   }
}
