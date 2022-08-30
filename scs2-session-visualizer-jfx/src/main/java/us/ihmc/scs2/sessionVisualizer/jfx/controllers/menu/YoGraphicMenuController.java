package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import static us.ihmc.scs2.sessionVisualizer.jfx.managers.NewTerrainVisualRequest.visible;
import static us.ihmc.scs2.sessionVisualizer.jfx.managers.NewTerrainVisualRequest.wireframeMode;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest.ALL_ROBOTS;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest.visible;
import static us.ihmc.scs2.sessionVisualizer.jfx.yoRobot.NewRobotVisualRequest.wireframeMode;

import java.io.File;

import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
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

   private JavaFXMessager messager;
   private SessionVisualizerTopics topics;
   private Stage owner;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      owner = toolkit.getWindow();

      messager.bindBidirectional(topics.getShowOverheadPlotter(), overheadPlotterMenuItem.selectedProperty(), false);
      showRobotMenuItem.selectedProperty()
                       .addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getRobotVisualRequest(), visible(ALL_ROBOTS, newValue)));
      enableWireframeRobotModeMenuItem.selectedProperty().addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getRobotVisualRequest(),
                                                                                                                        wireframeMode(ALL_ROBOTS, newValue)));
      showTerrainMenuItem.selectedProperty()
                         .addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getTerrainVisualRequest(), visible(newValue)));
      enableWireframeTerrainModeMenuItem.selectedProperty().addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getTerrainVisualRequest(),
                                                                                                                          wireframeMode(newValue)));
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
