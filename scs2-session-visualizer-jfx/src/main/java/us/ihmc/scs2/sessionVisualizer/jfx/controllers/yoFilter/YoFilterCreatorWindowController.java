package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoFilter;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
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
import javafx.scene.text.Font;
import javafx.scene.text.FontPosture;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import org.kordamp.ikonli.javafx.FontIcon;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.filtering.YoFilter;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator.YoEquationEditorHelpPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator.YoEquationEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoFilter.search.YoFilterListCell;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.IOException;
import java.util.function.Function;

/**
 * This class is based on {@link us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.creator.YoCompositeAndEquationEditorWindowController}
 */
public class YoFilterCreatorWindowController
{
   @FXML
   private Pane mainPane;
   @FXML
   private ListView<YoFilter> filterListView;

   private final Property<YoNameDisplay> yoVariableNameDisplay = new SimpleObjectProperty<>(this, "yoVariableNameDisplay", YoNameDisplay.SHORT_NAME);

   private Stage window;
   private SessionVisualizerToolkit toolkit;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private YoManager yoManager;
   private YoSharedBuffer yoSharedBuffer;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      yoManager = toolkit.getYoManager();
      yoSharedBuffer = toolkit.getSession().getBuffer();

      Property<Integer> numberPrecision = messager.createPropertyInput(topics.getControlsNumberPrecision(), 3);

      filterListView.setCellFactory(param -> new YoFilterListCell(yoManager, yoVariableNameDisplay, numberPrecision, param));
      filterListView.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);

      Function<ListView<YoFilter>, MenuItem> newYoVariable = listView ->
      {
         FontIcon addAfterIcon = new FontIcon();
         addAfterIcon.getStyleClass().add("add-icon-view");
         MenuItem menuItem = new MenuItem("New YoFilter", addAfterIcon);
         menuItem.setOnAction(e -> newYoFilter());
         return menuItem;
      };
      Function<ListView<YoFilter>, MenuItem> deleteYoVariable = listView ->
      {
         FontIcon removeIcon = new FontIcon();
         removeIcon.getStyleClass().add("remove-icon-view");
         MenuItem menuItem = new MenuItem("Delete YoFilter", removeIcon);
         menuItem.setOnAction(e -> deleteYoFilter(listView.getSelectionModel().getSelectedItem()));
         return menuItem;
      };
      MenuTools.setupContextMenu(filterListView, newYoVariable, deleteYoVariable);

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
      refreshYoFilterListView();
   }

   @FXML
   public void newYoFilter()
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_FILTER_CREATOR_DIALOG_URL);
         loader.load();
         YoFilterCreatorDialogController yoFilterCreatorDialogController = loader.getController();
         YoFilter yoFilter = yoFilterCreatorDialogController.showAndWait(window, yoManager.getFilterRegistry(), yoSharedBuffer);
         yoManager.getFilters().add(yoFilter);
         if (yoFilter != null)
         {
            refreshYoFilterListView();
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void deleteYoFilter(YoFilter selectedFilter)
   {
      if (selectedFilter == null)
         return;

      filterListView.getItems().remove(selectedFilter);

      for (YoVariable yoVariable : selectedFilter.getYoComponents())
      {
         yoVariable.destroy();
      }
   }

   private void refreshYoFilterListView()
   {
      filterListView.getItems().clear();
      yoManager.getFilters().forEach(filterListView.getItems()::add);
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
      window.close();
      filterListView.getItems().clear();
   }

   @FXML
   public void openHelpDialog()
   {
      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_FILTER_CREATOR_WINDOW_URL);
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
   public void exportFilters()
   {
      /*
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

       */
   }

   @FXML
   public void importFilters()
   {
      /*
      File file = SessionVisualizerIOTools.yoEquationOpenFileDialog(window);

      if (file == null)
         return;

      try (FileInputStream inputStream = new FileInputStream(file))
      {
         YoEquationListDefinition yoEquationListDefinition = DefinitionIOTools.loadYoEquationListDefinition(inputStream);
         if (yoEquationListDefinition == null)
            return;
         YoRegistry userRegistry = yoManager.getUserRegistry();
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

       */
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
