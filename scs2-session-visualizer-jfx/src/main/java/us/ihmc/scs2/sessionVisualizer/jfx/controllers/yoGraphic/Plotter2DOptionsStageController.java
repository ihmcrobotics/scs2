package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic;

import java.util.ArrayList;
import java.util.List;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;

public class Plotter2DOptionsStageController implements VisualizerController
{
   @FXML
   private Stage stage;
   @FXML
   private ToggleButton enableTrackingToggleButton;
   @FXML
   private YoCompositeEditorPaneController trackedPositionEditorController;

   private final List<Runnable> cleanupActions = new ArrayList<>();
   private Window owner;
   private YoCompositeSearchManager yoCompositeSearchManager;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      owner = toolkit.getWindow();
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();

      SCS2JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();
      Topic<YoTuple2DDefinition> trackCoordsTopic = topics.getPlotter2DTrackCoordinateRequest();
      Property<YoTuple2DDefinition> lastCoordinates = new SimpleObjectProperty<>(this, "lastCoordinatesProperty", messager.getLastValue(trackCoordsTopic));

      trackedPositionEditorController.initialize(toolkit.getGlobalToolkit(), yoCompositeSearchManager.getCollectionFromType(YoCompositeTools.YO_TUPLE2D), true);
      trackedPositionEditorController.setCompositeName("Tracking Coordinates");

      enableTrackingToggleButton.selectedProperty().addListener((o, oldValue, newValue) ->
      {
         if (newValue)
            messager.submitMessage(trackCoordsTopic, lastCoordinates.getValue());
         else
            messager.submitMessage(trackCoordsTopic, null);
      });

      trackedPositionEditorController.getMainPane().disableProperty().bind(enableTrackingToggleButton.selectedProperty().not());

      if (lastCoordinates.getValue() != null)
      {
         trackedPositionEditorController.setInput(lastCoordinates.getValue());
         enableTrackingToggleButton.selectedProperty().set(true);
      }

      trackedPositionEditorController.addInputListener((coords, frame) ->
      {
         YoTuple2DDefinition newCoordinates = CompositePropertyTools.toYoTuple2DDefinition(new Tuple2DProperty(frame, coords));
         lastCoordinates.setValue(newCoordinates);
         messager.submitMessage(trackCoordsTopic, newCoordinates);
      });

      EventHandler<? super WindowEvent> closeWindowEventHandler = e -> close();
      owner.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler);
      cleanupActions.add(() -> owner.removeEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, closeWindowEventHandler));

      stage.setOnCloseRequest(e -> close());
      SessionVisualizerIOTools.addSCSIconToWindow(stage);
      JavaFXMissingTools.centerWindowInOwner(stage, owner);
   }

   public Stage getStage()
   {
      return stage;
   }

   public void close()
   {
      stage.close();
      cleanupActions.forEach(Runnable::run);
      cleanupActions.clear();
   }
}
