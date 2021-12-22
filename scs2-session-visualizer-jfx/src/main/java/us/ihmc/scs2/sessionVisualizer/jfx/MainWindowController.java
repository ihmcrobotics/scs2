package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.IOException;
import java.time.Duration;

import com.jfoenix.controls.JFXDrawer;
import com.jfoenix.controls.JFXHamburger;
import com.jfoenix.controls.events.JFXDrawerEvent;
import com.jfoenix.transitions.hamburger.HamburgerSlideCloseTransition;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Cursor;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.control.Label;
import javafx.scene.control.SplitPane;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import us.ihmc.commons.Conversions;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionAdvancedControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionSimpleControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.YoChartGroupPanelController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu.MainWindowMenuBarController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.plotter.Plotter2D;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;

public class MainWindowController extends ObservedAnimationTimer
{
   @FXML
   private AnchorPane mainPane;
   @FXML
   private AnchorPane sceneAnchorPane;
   @FXML
   private SplitPane mainViewSplitPane;
   @FXML
   private JFXHamburger hamburger;
   @FXML
   private JFXDrawer toolDrawer;
   @FXML
   private Label fpsLabel;
   @FXML
   private MainWindowMenuBarController mainWindowMenuBarController;
   @FXML
   private SessionSimpleControlsController sessionSimpleControlsController;
   @FXML
   private SessionAdvancedControlsController sessionAdvancedControlsController;
   @FXML
   private YoChartGroupPanelController yoChartGroupPanelController;

   private SidePaneController sidePaneController;

   private SessionVisualizerToolkit globalToolkit;
   private SessionVisualizerWindowToolkit windowToolkit;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;

   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      windowToolkit = toolkit;
      this.globalToolkit = toolkit.getGlobalToolkit();
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();

      mainWindowMenuBarController.initialize(windowToolkit);
      sessionSimpleControlsController.initialize(windowToolkit);
      sessionAdvancedControlsController.initialize(windowToolkit);
      yoChartGroupPanelController.initialize(windowToolkit);

      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.SIDE_PANE_URL);
         Pane sidePane = setupDrawer(loader.load());
         messager.registerJavaFXSyncedTopicListener(topics.getDisableUserControls(), disable -> sidePane.setDisable(disable));
         sidePaneController = loader.getController();
         sidePaneController.initialize(toolkit.getGlobalToolkit());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   public void setupViewport3D(Pane viewportPane)
   {
      sceneAnchorPane.getChildren().set(0, viewportPane);
      AnchorPane.setTopAnchor(viewportPane, 0.0);
      AnchorPane.setRightAnchor(viewportPane, 0.0);
      AnchorPane.setBottomAnchor(viewportPane, 0.0);
      AnchorPane.setLeftAnchor(viewportPane, 0.0);
      globalToolkit.getSnapshotManager().registerRecordable(viewportPane);
   }

   private Property<Boolean> showOverheadPlotterProperty;

   public Property<Boolean> setupPlotter2D(Plotter2D plotter2D)
   {
      SubScene plotter2DScene = new SubScene(plotter2D, 100, 10);
      Pane pane = new Pane(plotter2DScene);
      plotter2DScene.heightProperty().bind(pane.heightProperty());
      plotter2DScene.widthProperty().bind(pane.widthProperty());
      plotter2D.getRoot().getChildren().add(globalToolkit.getYoGraphicFXManager().getRootNode2D());

      showOverheadPlotterProperty = messager.createPropertyInput(topics.getShowOverheadPlotter(), false);
      showOverheadPlotterProperty.addListener((o, oldValue, newValue) ->
      {
         if (newValue)
         {
            if (!mainViewSplitPane.getItems().contains(pane))
            {
               mainViewSplitPane.getItems().add(pane);
               plotter2D.setScale(400.0 / 5.0);
            }
         }
         else
         {
            mainViewSplitPane.getItems().remove(pane);
         }
      });
      return showOverheadPlotterProperty;
   }

   public Pane setupDrawer(Pane sidePane)
   {
      // Workaround for the drawer resizing:
      // Here we make an edge similar to a SplitPane separator on which the cursor will change
      StackPane edge = new StackPane();
      edge.setStyle("-fx-background-color:white;-fx-effect:innershadow(three-pass-box, #A9A9A9, 2.5, 0.0, -1.0, 0.0)");
      edge.setPrefWidth(5.0);
      edge.setMinWidth(5.0);
      edge.setCursor(Cursor.E_RESIZE);
      HBox hBox = new HBox(sidePane, edge);
      toolDrawer.setSidePane(hBox);
      HBox.setHgrow(sidePane, Priority.ALWAYS);
      HBox.setHgrow(edge, Priority.NEVER);
      // The edge passes on the MouseEvents used for resizing the drawer to the inner StackPane.
      edge.addEventHandler(MouseEvent.MOUSE_PRESSED, hBox.getParent()::fireEvent);
      edge.addEventHandler(MouseEvent.MOUSE_DRAGGED, hBox.getParent()::fireEvent);
      edge.addEventHandler(MouseEvent.MOUSE_RELEASED, hBox.getParent()::fireEvent);

      Node remove = toolDrawer.getChildren().remove(toolDrawer.getChildren().size() - 1);
      mainPane.addEventHandler(KeyEvent.KEY_PRESSED, (EventHandler<? super KeyEvent>) e ->
      {
         if (e.getCode() == KeyCode.ESCAPE && toolDrawer.isOpened())
         {
            toolDrawer.close();
            e.consume();
         }
      });
      toolDrawer.setResizeContent(true);
      toolDrawer.setResizableOnDrag(true);
      toolDrawer.setContent(remove);

      // By disabling the side pane, we unlink YoVariables (in search tabs) reducing the cost of a run tick for the Session
      hBox.setVisible(toolDrawer.isOpened());
      hBox.setDisable(!toolDrawer.isOpened());
      toolDrawer.addEventHandler(JFXDrawerEvent.ANY, e ->
      {
         if (e.getEventType() == JFXDrawerEvent.CLOSED)
         {
            hBox.setVisible(false);
            hBox.setDisable(true);
         }
         if (e.getEventType() == JFXDrawerEvent.OPENING || e.getEventType() == JFXDrawerEvent.OPENED)
         {
            hBox.setVisible(true);
            hBox.setDisable(false);
         }
      });

      hamburger.setOnMouseClicked(e ->
      {
         if (toolDrawer.isClosed() || toolDrawer.isClosing())
         {
            toolDrawer.open();
            // The event handler is not clean visually, need something that kicks in earlier.
            hBox.setVisible(true);
            hBox.setDisable(false);
         }
         else
         {
            toolDrawer.close();
         }
      });

      HamburgerSlideCloseTransition transition = new HamburgerSlideCloseTransition(hamburger);

      toolDrawer.addEventHandler(JFXDrawerEvent.CLOSING, e ->
      {
         transition.setRate(-0.5);
         transition.play();
      });
      toolDrawer.addEventFilter(JFXDrawerEvent.OPENING, e ->
      {
         transition.setRate(0.5);
         transition.play();
      });

      return hBox;
   }

   private long timeLast = -1;
   private long timeIntervalBetweenUpdates = Duration.ofMillis(500).toNanos();
   private int frameCounter = 0;
   private double goodFPSLowerThreshold = 40.0;
   private double mediumFPSLowerThreshold = 20.0;
   private Color goodFPSColor = Color.FORESTGREEN;
   private Color mediumFPSColor = Color.DARKORANGE;
   private Color poorFPSColor = Color.RED;

   @Override
   public void handleImpl(long timeNow)
   {
      if (timeLast == -1)
      {
         timeLast = timeNow;
         frameCounter = 0;
         return;
      }

      frameCounter++;

      if (timeNow - timeLast < timeIntervalBetweenUpdates)
         return;

      double framesPerSecond = frameCounter / Conversions.nanosecondsToSeconds(timeNow - timeLast);
      fpsLabel.setText(String.format("%6.2f FPS", framesPerSecond));
      if (framesPerSecond >= goodFPSLowerThreshold)
         fpsLabel.setTextFill(goodFPSColor);
      else if (framesPerSecond >= mediumFPSLowerThreshold)
         fpsLabel.setTextFill(mediumFPSColor);
      else
         fpsLabel.setTextFill(poorFPSColor);

      timeLast = timeNow;
      frameCounter = 0;
   }

   @Override
   public void start()
   {
      super.start();
      windowToolkit.start();
   }

   @Override
   public void stop()
   {
      stopSession();
      super.stop();
      windowToolkit.stop();
   }

   public void startSession()
   {
      yoChartGroupPanelController.start();
      sidePaneController.start();
   }

   public void stopSession()
   {
      yoChartGroupPanelController.closeAndDispose();
      sidePaneController.stop();
   }

   public AnchorPane getMainPane()
   {
      return mainPane;
   }

   public AnchorPane getSceneAnchorPane()
   {
      return sceneAnchorPane;
   }

   public SplitPane getMainViewSplitPane()
   {
      return mainViewSplitPane;
   }

   public YoChartGroupPanelController getYoChartGroupPanelController()
   {
      return yoChartGroupPanelController;
   }

   public SidePaneController getSidePaneController()
   {
      return sidePaneController;
   }

   public Property<Boolean> showOverheadPlotterProperty()
   {
      return showOverheadPlotterProperty;
   }

   public BooleanProperty showAdvancedControlsProperty()
   {
      return sessionAdvancedControlsController.showProperty();
   }
}
