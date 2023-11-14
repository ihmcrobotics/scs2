package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator;

import com.jfoenix.controls.JFXTextField;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.ListChangeListener;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.WindowEvent;
import javafx.util.Callback;
import javafx.util.Duration;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.messager.Messager;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.YoCompositeListCell;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.symbolic.YoEquationManager.YoEquationListChange;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;
import java.util.*;
import java.util.function.Function;
import java.util.function.Predicate;

public class YoCompositeCreatorWindowController
{
   private static final String NEW_COMPOSITE_NAME = "NewVariable";

   @FXML
   private Pane mainPane;
   @FXML
   private ListView<YoComposite> yoCompositeListView;
   @FXML
   private ListView<YoEquationCreatorController> yoEquationListView;

   private final Property<YoNameDisplay> yoVariableNameDisplay = new SimpleObjectProperty<>(this, "yoVariableNameDisplay", YoNameDisplay.SHORT_NAME);
   private YoRegistry userRegistry;
   private YoCompositeSearchManager yoCompositeSearchManager;

   private Stage window;
   private SessionVisualizerToolkit toolkit;

   private List<YoEquationDefinition> yoEquationDefinitions;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();
      YoManager yoManager = toolkit.getYoManager();
      userRegistry = yoManager.getUserRegistry();
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();

      Property<Integer> numberPrecision = messager.createPropertyInput(topics.getControlsNumberPrecision(), 3);

      yoCompositeListView.setCellFactory(param -> new YoCompositeListCell(yoManager, yoVariableNameDisplay, numberPrecision, param));
      yoCompositeListView.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);

      Function<ListView<YoComposite>, MenuItem> newYoVariable = listView ->
      {
         MenuItem menuItem = new MenuItem("New YoVariable");
         menuItem.setOnAction(e -> newYoVariableDialog(yoManager));
         return menuItem;
      };
      MenuTools.setupContextMenu(yoCompositeListView, newYoVariable);

      yoEquationListView.setCellFactory(new Callback<>()
      {
         @Override
         public ListCell<YoEquationCreatorController> call(ListView<YoEquationCreatorController> param)
         {
            return new ListCell<>()
            {
               @Override
               protected void updateItem(YoEquationCreatorController item, boolean empty)
               {
                  super.updateItem(item, empty);
                  setGraphic(item == null ? null : item.getMainPane());
               }
            };
         }
      });

      yoEquationDefinitions = toolkit.getSession().getYoEquationDefinitions();

      for (YoEquationDefinition equationDefinition : yoEquationDefinitions)
      {
         newEquationController(equationDefinition);
      }

      MutableBoolean isUpdating = new MutableBoolean(false);

      messager.addFXTopicListener(topics.getSessionYoEquationListChangeState(), m ->
      {
         isUpdating.setTrue();

         for (int i = 0; i < m.getEquations().size(); i++)
         {
            YoEquationDefinition equationDefinition = m.getEquations().get(i);
            YoEquationCreatorController controller = yoEquationListView.getItems().get(i);
            controller.setEquationDefinition(equationDefinition);
         }

         yoEquationListView.getItems().remove(m.getEquations().size(), yoEquationListView.getItems().size());
         isUpdating.setFalse();
      });

      yoEquationListView.getItems().addListener((ListChangeListener<YoEquationCreatorController>) c ->
      {
         updateEquationsAndSend(isUpdating);
         c.getList().forEach(controller -> controller.setUpdateListener(definition -> updateEquationsAndSend(isUpdating)));
      });

      MenuTools.setupContextMenu(yoEquationListView, ListViewTools.removeMenuItemFactory(false));

      window = new Stage(StageStyle.UTILITY);
      window.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ESCAPE)
            window.close();
      });
      toolkit.getMainWindow().addEventFilter(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            window.close();
      });
      //TODO: Fix this
      //      window.setOnHidden(e -> stop());
      //      window.setOnShowing(e -> start());
      window.setTitle("YoGraphic properties");
      window.setScene(new Scene(mainPane));
      window.initOwner(toolkit.getMainWindow());
   }

   private void updateEquationsAndSend(MutableBoolean isUpdating)
   {
      if (isUpdating.isTrue())
         return;

      Messager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      isUpdating.setTrue();
      // Name set to verify no duplicate names
      Set<String> equationNameSet = new HashSet<>();
      yoEquationDefinitions = new ArrayList<>();
      boolean areAllValid = true;
      for (YoEquationCreatorController controller : yoEquationListView.getItems())
      {
         yoEquationDefinitions.add(controller.getEquationDefinition());
         if (!controller.validityProperty().get())
            areAllValid = false;
         if (!equationNameSet.add(controller.getEquationDefinition().getName()))
            areAllValid = false; // Duplicate name
      }
      if (areAllValid)
         messager.submitMessage(topics.getSessionYoEquationListChangeRequest(), YoEquationListChange.newList(yoEquationDefinitions));
      isUpdating.setFalse();
   }

   private void newYoVariableDialog(YoManager yoManager)
   {
      Dialog<ButtonType> dialog = new Dialog<>();
      dialog.setContentText("Enter YoVariable name");
      JFXTextField myVariableTextField = new JFXTextField("myVariable");
      dialog.setGraphic(myVariableTextField);
      dialog.setTitle("New YoVariable");
      dialog.getDialogPane().getButtonTypes().addAll(ButtonType.OK, ButtonType.CANCEL);
      Optional<ButtonType> result = dialog.showAndWait();
      if (result.isPresent() && result.get() == ButtonType.OK)
      {
         try
         {
            new YoDouble(myVariableTextField.getText(), yoManager.getUserRegistry());
            refreshYoCompositeListView();
         }
         catch (Exception e)
         {
         }
      }
   }

   private void refreshYoCompositeListView()
   {
      yoCompositeListView.getItems().clear();
      userRegistry.getVariables()
                  .forEach(yoVariable -> yoCompositeListView.getItems().add(new YoComposite(YoCompositeSearchManager.yoVariablePattern, yoVariable)));
   }

   @FXML
   public void newEquation(ActionEvent actionEvent)
   {
      newEquationController(new YoEquationDefinition());
   }

   private void newEquationController(YoEquationDefinition definition)
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_EQUATION_CREATOR_PANE_URL);
         loader.load();
         YoEquationCreatorController controller = loader.getController();
         Predicate<String> nameValidator = name -> yoEquationListView.getItems()
                                                                     .stream()
                                                                     .noneMatch(c -> c != controller && c.getEquationDefinition().getName().equals(name));
         controller.initialize(toolkit, nameValidator, definition);
         yoEquationListView.getItems().add(controller);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public Pane getMainPane()
   {
      return mainPane;
   }

   public void showWindow()
   {
      window.setOpacity(0.0);
      window.toFront();
      window.show();
      Timeline timeline = new Timeline();
      KeyFrame key = new KeyFrame(Duration.seconds(0.125), new KeyValue(window.opacityProperty(), 1.0));
      timeline.getKeyFrames().add(key);
      timeline.play();
   }

   public Stage getWindow()
   {
      return window;
   }

   public void closeAndDispose()
   {
   }

   public void startSession(Session session)
   {
   }

   public void stopSession()
   {
   }
}
