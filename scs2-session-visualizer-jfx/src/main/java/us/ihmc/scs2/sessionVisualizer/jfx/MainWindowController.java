package us.ihmc.scs2.sessionVisualizer.jfx;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import org.apache.commons.lang3.mutable.MutableBoolean;

import com.jfoenix.controls.JFXDrawer;
import com.jfoenix.controls.JFXDrawer.DrawerDirection;
import com.jfoenix.controls.JFXHamburger;
import com.jfoenix.controls.events.JFXDrawerEvent;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.event.Event;
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
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.HamburgerAnimationTransition.FrameType;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionAdvancedControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.SessionSimpleControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.YoChartGroupPanelController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu.MainWindowMenuBarController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.plotter.Plotter2D;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableDatabase;

public class MainWindowController extends ObservedAnimationTimer implements VisualizerController
{
   @FXML
   private AnchorPane rootPane;
   @FXML
   private SplitPane mainGUIPane;
   @FXML
   private AnchorPane sceneAnchorPane;
   @FXML
   private SplitPane mainViewSplitPane;
   @FXML
   private JFXHamburger leftDrawerBurger, rightDrawerBurger;
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

   /** The drawer used to hold onto the tools for searching yoVariables. */
   private final JFXDrawer leftDrawer = new JFXDrawer();
   /** The drawer used to hold onto custom GUI controls. */
   private final JFXDrawer rightDrawer = new JFXDrawer();

   private final BooleanProperty leftDrawerOpen = new SimpleBooleanProperty(this, "leftDrawerOpenProperty", false);
   private final BooleanProperty rightDrawerOpen = new SimpleBooleanProperty(this, "rightDrawerOpenProperty", false);
   private final BooleanProperty disableUserControls = new SimpleBooleanProperty(this, "disableUserControlsProperty", false);

   /** Controller for the left pane where variable search and entries are displayed. */
   private SidePaneController sidePaneController;
   /** Controller for the right pane where custom user controls are displayed. */
   private UserSidePaneController userSidePaneController;

   private final Plotter2D plotter2D = new Plotter2D();

   private SessionVisualizerToolkit globalToolkit;
   private SessionVisualizerWindowToolkit windowToolkit;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      windowToolkit = toolkit;
      globalToolkit = toolkit.getGlobalToolkit();
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();

      messager.addFXTopicListener(topics.getDisableUserControls(), m -> disableUserControls.set(m));

      mainWindowMenuBarController.initialize(windowToolkit);
      sessionSimpleControlsController.initialize(windowToolkit);
      sessionAdvancedControlsController.initialize(windowToolkit);
      // Show the advanced controls by default
      sessionSimpleControlsController.show(false);
      sessionAdvancedControlsController.showProperty().set(true);
      yoChartGroupPanelController.initialize(windowToolkit);

      rootPane.getChildren().set(1, leftDrawer);
      JavaFXMissingTools.setAnchorConstraints(leftDrawer, 25, 0, 0, 0);
      leftDrawer.getChildren().add(rightDrawer);
      leftDrawer.setDefaultDrawerSize(300);
      leftDrawer.setDirection(DrawerDirection.LEFT);

      rightDrawer.getChildren().add(mainGUIPane);
      rightDrawer.setDirection(DrawerDirection.RIGHT);
      rightDrawer.setDefaultDrawerSize(300);

      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.SIDE_PANE_URL);
         setupLeftDrawer(loader.load());
         sidePaneController = loader.getController();
         sidePaneController.initialize(toolkit.getGlobalToolkit());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.USER_SIDE_PANE_URL);
         setupRightDrawer(loader.load());
         userSidePaneController = loader.getController();
         userSidePaneController.initialize(toolkit);
         userSidePaneController.computedPrefWidthProperty().addListener((o, oldValue, newValue) -> rightDrawer.setDefaultDrawerSize(newValue.doubleValue()));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      setupPlotter2D(plotter2D);
      messager.addFXTopicListener(topics.getPlotter2DTrackCoordinateRequest(), m ->
      {
         YoVariableDatabase rootRegistryDatabase = toolkit.getYoManager().getRootRegistryDatabase();
         ReferenceFrameManager referenceFrameManager = toolkit.getReferenceFrameManager();
         plotter2D.coordinateToTrackProperty().setValue(CompositePropertyTools.toTuple2DProperty(rootRegistryDatabase, referenceFrameManager, m));
      });
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

   public void setupLeftDrawer(Pane sidePane)
   {
      configureDrawer(sidePane, leftDrawer, leftDrawerBurger, leftDrawerOpen);
      HamburgerAnimationTransition transition = new HamburgerAnimationTransition(leftDrawerBurger, FrameType.BURGER, FrameType.LEFT_CLOSE);

      leftDrawer.addEventHandler(Event.ANY, e ->
      {
         if (e.getTarget() != leftDrawer)
            return;

         if (e.getEventType() == JFXDrawerEvent.OPENED)
         {
            sidePaneController.getYoSearchTabPaneController().requestFocusForActiveSearchBox();
         }
         else if (e.getEventType() == JFXDrawerEvent.CLOSING)
         {
            transition.setRate(-1);
            transition.play();
         }
         else if (e.getEventType() == JFXDrawerEvent.OPENING)
         {
            transition.setRate(1);
            transition.play();
         }
      });
   }

   public void setupRightDrawer(Pane sidePane)
   {
      configureDrawer(sidePane, rightDrawer, rightDrawerBurger, rightDrawerOpen);
      HamburgerAnimationTransition transition = new HamburgerAnimationTransition(rightDrawerBurger, FrameType.LEFT_ANGLE, FrameType.RIGHT_CLOSE);

      rightDrawer.addEventHandler(Event.ANY, e ->
      {
         if (e.getTarget() != rightDrawer)
            return;

         if (e.getEventType() == JFXDrawerEvent.OPENED)
         {
            // TODO
         }
         else if (e.getEventType() == JFXDrawerEvent.CLOSING)
         {
            transition.setRate(-1);
            transition.play();
         }
         else if (e.getEventType() == JFXDrawerEvent.OPENING)
         {
            transition.setRate(1);
            transition.play();
         }
      });
   }

   public void configureDrawer(Pane sidePane, JFXDrawer drawer, Node openCloseControl, BooleanProperty drawerOpenProperty)
   {
      Pane drawerSidePane = addEdgeToSidePane(sidePane, drawer.getDirection());
      drawer.setSidePane(drawerSidePane);

      Node remove = drawer.getChildren().remove(drawer.getChildren().size() - 1);
      rootPane.addEventHandler(KeyEvent.KEY_PRESSED, (EventHandler<? super KeyEvent>) e ->
      {
         if (e.isConsumed())
            return;
         if (e.getCode() == KeyCode.ESCAPE && drawer.isOpened())
         {
            drawer.close();
            e.consume();
         }
      });
      drawer.setResizeContent(true);
      drawer.setResizableOnDrag(true);
      drawer.setOverLayVisible(false);
      drawer.setContent(remove);

      BooleanProperty drawerFullyClosed = new SimpleBooleanProperty(drawer, "drawerFullyClosedProperty", !drawerOpenProperty.get());
      drawerOpenProperty.set(drawer.isOpened());

      MutableBoolean ignoreOpenChange = new MutableBoolean(false);

      drawerOpenProperty.addListener((o, oldValue, newValue) ->
      {
         if (ignoreOpenChange.booleanValue())
            return;
         if (newValue && (drawer.isClosing() || drawer.isClosed()))
            drawer.open();
         else if (drawer.isOpening() || drawer.isOpened())
            drawer.close();
      });

      drawer.addEventHandler(Event.ANY, e ->
      {
         if (e.getTarget() != drawer)
            return;

         if (e.getEventType() == JFXDrawerEvent.CLOSED)
         {
            ignoreOpenChange.setTrue();
            drawerOpenProperty.set(false);
            ignoreOpenChange.setFalse();
            drawerFullyClosed.set(true);
         }
         if (e.getEventType() == JFXDrawerEvent.OPENING || e.getEventType() == JFXDrawerEvent.OPENED)
         {
            ignoreOpenChange.setTrue();
            drawerOpenProperty.set(true);
            ignoreOpenChange.setFalse();
            drawerFullyClosed.set(false);
         }
      });

      openCloseControl.setOnMouseClicked(e ->
      {
         if (drawer.isClosed() || drawer.isClosing())
            drawer.open();
         else
            drawer.close();
      });

      // By disabling the side pane, we unlink YoVariables (in search tabs) reducing the cost of a run tick for the Session
      drawerSidePane.visibleProperty().bind(drawerFullyClosed.not());
      drawerSidePane.disableProperty().bind(drawerFullyClosed.or(disableUserControls));
   }

   private Pane addEdgeToSidePane(Pane sidePane, DrawerDirection contentSide)
   {
      if (contentSide == DrawerDirection.LEFT || contentSide == DrawerDirection.RIGHT)
      {
         // Workaround for the drawer resizing:
         // Here we make an edge similar to a SplitPane separator on which the cursor will change
         StackPane edge = new StackPane();
         edge.setStyle("-fx-background-color:white;-fx-effect:innershadow(three-pass-box, #A9A9A9, 2.5, 0.0, -1.0, 0.0)");
         edge.setPrefWidth(5.0);
         edge.setMinWidth(5.0);
         edge.setCursor(Cursor.E_RESIZE);
         HBox drawerSidePane;
         if (contentSide == DrawerDirection.LEFT)
            drawerSidePane = new HBox(sidePane, edge);
         else
            drawerSidePane = new HBox(edge, sidePane);

         // The edge passes on the MouseEvents used for resizing the drawer to the inner StackPane.
         edge.addEventHandler(MouseEvent.ANY, event ->
         {
            if (drawerSidePane.getParent() != null)
               drawerSidePane.getParent().fireEvent(event);
         });
         HBox.setHgrow(sidePane, Priority.ALWAYS);
         HBox.setHgrow(edge, Priority.NEVER);
         return drawerSidePane;
      }

      throw new UnsupportedOperationException("Not implemented for: " + contentSide);
   }

   private long timeLast = -1;
   private long timeIntervalBetweenUpdates = TimeUnit.MILLISECONDS.toNanos(500);
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
      plotter2D.coordinateToTrackProperty().setValue(null);
   }

   public AnchorPane getMainPane()
   {
      return rootPane;
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

   public UserSidePaneController getUserSidePaneController()
   {
      return userSidePaneController;
   }

   public BooleanProperty leftSidePaneOpenProperty()
   {
      return leftDrawerOpen;
   }

   public BooleanProperty rightSidePaneOpenProperty()
   {
      return rightDrawerOpen;
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
