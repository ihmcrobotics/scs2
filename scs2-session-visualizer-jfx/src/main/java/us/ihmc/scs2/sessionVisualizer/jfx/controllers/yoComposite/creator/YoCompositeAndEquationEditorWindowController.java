package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;
import javafx.scene.control.SelectionMode;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import javafx.scene.text.FontPosture;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import org.kordamp.ikonli.javafx.FontIcon;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.yoVariable.YoEquationDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEquationListDefinition;
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
import us.ihmc.scs2.symbolic.YoEquationManager;
import us.ihmc.scs2.symbolic.YoEquationManager.YoEquationListChange;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

public class YoCompositeAndEquationEditorWindowController
{
   public static final String NEW_EQUATION_NAME = "NewEquation";

   @FXML
   private Pane mainPane;
   @FXML
   private ListView<YoComposite> yoCompositeListView;
   @FXML
   private ListView<YoEquationEditorPaneController> yoEquationEditorListView;
   @FXML
   private VBox equationEditorContainer;

   private final Property<YoNameDisplay> yoVariableNameDisplay = new SimpleObjectProperty<>(this, "yoVariableNameDisplay", YoNameDisplay.SHORT_NAME);
   private YoRegistry userRegistry;

   private Stage window;
   private SessionVisualizerToolkit toolkit;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      YoManager yoManager = toolkit.getYoManager();
      userRegistry = yoManager.getUserRegistry();

      Property<Integer> numberPrecision = messager.createPropertyInput(topics.getControlsNumberPrecision(), 3);

      yoCompositeListView.setCellFactory(param -> new YoCompositeListCell(yoManager, yoVariableNameDisplay, numberPrecision, param));
      yoCompositeListView.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);

      Function<ListView<YoComposite>, MenuItem> newYoVariable = listView ->
      {
         FontIcon addAfterIcon = new FontIcon();
         addAfterIcon.getStyleClass().add("add-icon-view");
         MenuItem menuItem = new MenuItem("New YoComposite", addAfterIcon);
         menuItem.setOnAction(e -> newYoComposite());
         return menuItem;
      };
      Function<ListView<YoComposite>, MenuItem> deleteYoVariable = listView ->
      {
         FontIcon removeIcon = new FontIcon();
         removeIcon.getStyleClass().add("remove-icon-view");
         MenuItem menuItem = new MenuItem("Delete YoComposite", removeIcon);
         menuItem.setOnAction(e -> deleteYoComposite(listView.getSelectionModel().getSelectedItem()));
         return menuItem;
      };
      MenuTools.setupContextMenu(yoCompositeListView, newYoVariable, deleteYoVariable);

      yoEquationEditorListView.setCellFactory(param -> new YoEquationListCell());
      toolkit.getSession().getYoEquationDefinitions().forEach(this::newEquation);
      yoEquationEditorListView.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);
      yoEquationEditorListView.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
                                                                                      {
                                                                                         if (newValue == null)
                                                                                            hideEquationEditorPane();
                                                                                         else
                                                                                            showEquationEditorPane(newValue);
                                                                                      });

      messager.addFXTopicListener(topics.getSessionYoEquationListChangeState(), m ->
      {
         if (yoEquationEditorListView.getSelectionModel().getSelectedIndex() >= m.getEquations().size())
            yoEquationEditorListView.getSelectionModel().clearSelection();

         for (int i = 0; i < m.getEquations().size(); i++)
         {
            YoEquationDefinition equationDefinition = m.getEquations().get(i);
            YoEquationEditorPaneController controller = yoEquationEditorListView.getItems().get(i);
            controller.definitionProperty().setValue(equationDefinition);
         }

         yoEquationEditorListView.getItems().remove(m.getEquations().size(), yoEquationEditorListView.getItems().size());
      });

      MenuTools.setupContextMenu(yoEquationEditorListView, ListViewTools.removeMenuItemFactory(false));

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
      window.setTitle("YoEquation editor");
      window.setScene(new Scene(mainPane));
      window.initOwner(toolkit.getMainWindow());
      refreshYoCompositeListView();
   }

   private void showEquationEditorPane(YoEquationEditorPaneController controller)
   {
      if (equationEditorContainer.getChildren().size() == 1)
         equationEditorContainer.getChildren().add(controller.getMainPane());
      else
         equationEditorContainer.getChildren().set(1, controller.getMainPane());
      VBox.setVgrow(controller.getMainPane(), Priority.ALWAYS);
   }

   private void hideEquationEditorPane()
   {
      while (equationEditorContainer.getChildren().size() > 1)
         equationEditorContainer.getChildren().remove(1);
   }

   @FXML
   public void newYoComposite()
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_COMPOSITE_CREATOR_DIALOG_URL);
         loader.load();
         YoCompositeCreatorDialogController yoCompositeCreatorDialogController = loader.getController();
         YoComposite yoComposite = yoCompositeCreatorDialogController.showAndWait(window, userRegistry);
         if (yoComposite != null)
         {
            refreshYoCompositeListView();
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void deleteYoComposite(YoComposite selectedComposite)
   {
      if (selectedComposite == null)
         return;
      for (YoVariable yoVariable : selectedComposite.getYoComponents())
      {
         yoVariable.destroy();
      }
   }

   private void refreshYoCompositeListView()
   {
      yoCompositeListView.getItems().clear();
      userRegistry.getVariables()
                  .forEach(yoVariable -> yoCompositeListView.getItems().add(new YoComposite(YoCompositeSearchManager.yoVariablePattern, yoVariable)));
   }

   @FXML
   public void newEquation()
   {
      YoEquationDefinition newEquation = new YoEquationDefinition();
      newEquation.setName(NEW_EQUATION_NAME);
      newEquation.setEquation("a = b + c");
      int index = 0;
      while (!isEquationNameUnique(newEquation))
         newEquation.setName(NEW_EQUATION_NAME + "(%d)".formatted(index++));
      newEquation(newEquation);
      yoEquationEditorListView.getSelectionModel().selectLast();
   }

   public void newEquation(YoEquationDefinition equation)
   {
      YoEquationEditorPaneController yoEquationEditorPaneController;
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_EQUATION_EDITOR_PANE_URL);
         loader.load();
         yoEquationEditorPaneController = loader.getController();
         yoEquationEditorPaneController.initialize(toolkit, this::isEquationNameUnique);
         yoEquationEditorListView.getItems().add(yoEquationEditorPaneController);
         yoEquationEditorPaneController.setUpdateListener(() ->
                                                          {
                                                             if (areAllEquationsValid())
                                                                messager.submitMessage(topics.getSessionYoEquationListChangeRequest(),
                                                                                       YoEquationListChange.newList(collectEquationDefinitions()));
                                                          });
         yoEquationEditorPaneController.definitionProperty().setValue(equation);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private boolean areAllEquationsValid()
   {
      ObservableList<YoEquationEditorPaneController> items = yoEquationEditorListView.getItems();
      return !items.isEmpty() && items.stream().allMatch(item -> item.validityProperty().get());
   }

   private boolean isEquationNameUnique(YoEquationDefinition query)
   {
      return yoEquationEditorListView.getItems()
                                     .stream()
                                     .map(YoEquationEditorPaneController::getDefinition)
                                     .noneMatch(equation -> equation != query && Objects.equals(equation.getName(), query.getName()));
   }

   private List<YoEquationDefinition> collectEquationDefinitions()
   {
      return yoEquationEditorListView.getItems().stream().map(YoEquationEditorPaneController::getDefinition).toList();
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

   @FXML
   public void openHelpDialog()
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_EQUATION_EDITOR_HELP_PANE_URL);
         loader.load();
         YoEquationEditorHelpPaneController controller = loader.getController();
         controller.show(window);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @FXML
   public void exportYoEquations()
   {
      File file = SessionVisualizerIOTools.yoEquationSaveFileDialog(window);

      if (file == null)
         return;

      try (FileOutputStream outputStream = new FileOutputStream(file))
      {
         DefinitionIOTools.saveYoEquationListDefinition(outputStream, collectEquationDefinitions());
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @FXML
   public void importYoEquations()
   {
      File file = SessionVisualizerIOTools.yoEquationOpenFileDialog(window);

      if (file == null)
         return;

      try (FileInputStream inputStream = new FileInputStream(file))
      {
         YoEquationListDefinition yoEquationListDefinition = DefinitionIOTools.loadYoEquationListDefinition(inputStream);
         if (yoEquationListDefinition == null)
            return;
         for (YoEquationDefinition yoEquationDefinition : yoEquationListDefinition.getYoEquations())
         {
            YoEquationManager.ensureUserAliasesExist(yoEquationDefinition, userRegistry);
            newEquation(yoEquationDefinition);
            refreshYoCompositeListView();
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private static class YoEquationListCell extends ListCell<YoEquationEditorPaneController>
   {
      private final Font equationFont = Font.font(Font.getDefault().getFamily(), FontPosture.ITALIC, 12);

      @Override
      protected void updateItem(YoEquationEditorPaneController item, boolean empty)
      {
         super.updateItem(item, empty);
         textProperty().unbind();

         if (empty || item == null)
         {
            setGraphic(null);
            setText(null);
         }
         else
         {
            HBox graphic = new HBox(10);
            Label equationNameLabel = new Label();
            equationNameLabel.textProperty().bind(item.getEquationNameTextField().textProperty());
            graphic.getChildren().add(equationNameLabel);

            Label equationLabel = new Label();
            equationLabel.setFont(equationFont);
            equationLabel.textProperty().bind(item.getEquationTextArea().textProperty());
            graphic.getChildren().add(equationLabel);

            setGraphic(graphic);
         }
      }
   }
}
