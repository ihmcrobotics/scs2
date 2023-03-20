package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.pattern;

import static us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools.addAfterMenuItemFactory;
import static us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools.addBeforeMenuItemFactory;
import static us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools.removeMenuItemFactory;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import com.jfoenix.controls.JFXButton;

import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.SetChangeListener;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.yoComposite.YoCompositePatternDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositePattern;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;

public class YoCompositePatternPropertyWindowController
{
   private static final String NEW_PATTERN_NAME = "New Pattern";

   @FXML
   private AnchorPane mainAnchorPane;
   @FXML
   private JFXButton addPatternButton, removePatternButton;
   @FXML
   private ListView<ObjectProperty<YoCompositePatternDefinition>> yoCompositePatternListView;
   @FXML
   private JFXButton exportButton, importButton;

   @FXML
   private AnchorPane yoCompositePatternEditorPane;

   @FXML
   private JFXButton saveChangesButton, revertChangesButton;

   private final ObjectProperty<YoCompositePatternEditorController> activeEditor = new SimpleObjectProperty<>(this, "activeEditor", null);
   private final Map<ObjectProperty<YoCompositePatternDefinition>, YoCompositePatternEditorController> cachedEditors = new HashMap<>();
   private YoCompositeSearchManager yoCompositeSearchManager;

   private Stage window;
   private SessionVisualizerTopics topics;
   private JavaFXMessager messager;
   private SessionVisualizerToolkit toolkit;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      topics = toolkit.getTopics();
      messager = toolkit.getMessager();
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();
      yoCompositePatternListView.setCellFactory(param -> new ListCell<ObjectProperty<YoCompositePatternDefinition>>()
      {
         private final ObjectProperty<YoCompositePatternDefinition> cellValueProperty = new SimpleObjectProperty<>(this, "cellValue", null);

         {
            cellValueProperty.addListener((o, oldValue, newValue) ->
            {
               if (newValue == null)
                  setText(null);
               else
                  setText(newValue.getName());
            });
         }

         @Override
         protected void updateItem(ObjectProperty<YoCompositePatternDefinition> item, boolean empty)
         {
            super.updateItem(item, empty);

            if (empty || item == null)
            {
               cellValueProperty.unbind();
               cellValueProperty.set(null);
            }
            else
            {
               cellValueProperty.bind(item);
            }
         }
      });
      yoCompositePatternListView.getSelectionModel().selectedItemProperty()
                                .addListener((o, oldValue, newValue) -> processListSelectionUpdate(oldValue, newValue));

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
      window.setTitle("YoCompositePattern properties");
      window.setScene(new Scene(mainAnchorPane));
      window.initOwner(toolkit.getMainWindow());

      List<YoCompositePatternDefinition> customYoCompositePatterns = YoCompositeTools.toYoCompositePatternDefinitions(yoCompositeSearchManager.customYoCompositePatterns());

      yoCompositePatternListView.getItems().clear();
      for (YoCompositePatternDefinition yoCompositePattern : customYoCompositePatterns)
         yoCompositePatternListView.getItems().add(new SimpleObjectProperty<YoCompositePatternDefinition>(yoCompositePattern));

      yoCompositeSearchManager.customYoCompositePatterns().addListener((SetChangeListener<YoCompositePattern>) change ->
      {
         if (change.wasAdded())
            updateOrAddPattern(change.getElementAdded());
         if (change.wasRemoved())
            removePattern(change.getElementRemoved());
      });
      MenuTools.setupContextMenu(yoCompositePatternListView,
                                        addBeforeMenuItemFactory(this::newEmptyPattern),
                                        addAfterMenuItemFactory(this::newEmptyPattern),
                                        removeMenuItemFactory(false));

      saveChangesButton.setDisable(true);
      revertChangesButton.setDisable(true);

      activeEditor.addListener((observable, oldValue, newValue) ->
      {
         saveChangesButton.disableProperty().unbind();
         revertChangesButton.disableProperty().unbind();

         if (newValue == null)
         {
            saveChangesButton.setDisable(true);
            revertChangesButton.setDisable(true);
         }
         else
         {
            saveChangesButton.disableProperty().bind(newValue.hasChangesPendingProperty().and(newValue.inputsValidityProperty()).not());
            revertChangesButton.disableProperty().bind(newValue.hasChangesPendingProperty().not());
         }
      });

      yoCompositePatternEditorPane.addEventHandler(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.CONTROL)
            return;
         if (e.isControlDown())
         {
            if (e.getCode() == KeyCode.S && !saveChangesButton.isDisabled())
            {
               saveChangesButton.fire();
            }
            else if (e.getCode() == KeyCode.Z && !revertChangesButton.isDisabled())
            {
               revertChangesButton.fire();
            }
         }
      });
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

   public void closeAndDispose()
   {
      window.close();

      if (activeEditor.get() != null)
         activeEditor.get().closeAndDispose();
      cachedEditors.values().forEach(YoCompositePatternEditorController::closeAndDispose);
   }

   private boolean ignoreTreeSelectionUpdate = false;

   private void processListSelectionUpdate(ObjectProperty<YoCompositePatternDefinition> oldSelectedValue,
                                           ObjectProperty<YoCompositePatternDefinition> newSelectedValue)
   {
      if (ignoreTreeSelectionUpdate)
         return;

      if (shouldCancelAction(oldSelectedValue))
         return;

      unloadEditor();

      if (newSelectedValue == null)
         return;

      YoCompositePatternEditorController controller = cachedEditors.get(newSelectedValue);

      if (controller == null)
      {
         try
         {
            FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_COMPOSITE_PATTERN_EDITOR_PANE_URL);
            loader.load();
            controller = loader.getController();
            controller.initialize(toolkit);
            controller.setInput(newSelectedValue.get());
            cachedEditors.put(newSelectedValue, controller);
            // We're dealing with a new pattern, let's editing its name
            YoCompositePatternEditorController controllerFinal = controller;
            JavaFXMissingTools.runNFramesLater(1, () -> controllerFinal.startEditingCompositePatternName());
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      if (controller != null)
      {
         Pane editorPane = controller.getMainPane();
         controller.setNameOfOtherPatterns(yoCompositePatternListView.getItems().stream().filter(item -> item != newSelectedValue)
                                                                     .map(item -> item.get().getName()).collect(Collectors.toList()));
         activeEditor.set(controller);
         yoCompositePatternEditorPane.getChildren().add(editorPane);
         AnchorPane.setLeftAnchor(editorPane, 0.0);
         AnchorPane.setRightAnchor(editorPane, 0.0);
         AnchorPane.setTopAnchor(editorPane, 0.0);
         AnchorPane.setBottomAnchor(editorPane, 0.0);
         newSelectedValue.bind(controller.patternDefinitionProperty());
      }
   }

   private boolean shouldCancelAction(ObjectProperty<YoCompositePatternDefinition> itemToSelectOnCancel)
   {
      if (activeEditor.get() != null && activeEditor.get().hasChangesPending())
      {
         Alert alert = new Alert(AlertType.CONFIRMATION, "Do you want to discard the changes?", ButtonType.YES, ButtonType.NO);
         SessionVisualizerIOTools.addSCSIconToDialog(alert);
         Optional<ButtonType> result = alert.showAndWait();
         ButtonType answer = result.get();

         if (answer == ButtonType.YES)
         {
            cancelChanges();
            return false;
         }
         else if (answer == ButtonType.NO)
         {
            if (itemToSelectOnCancel != null)
            {
               JavaFXMissingTools.runLater(getClass(), () ->
               {
                  ignoreTreeSelectionUpdate = true;
                  yoCompositePatternListView.getSelectionModel().clearSelection();
                  yoCompositePatternListView.getSelectionModel().select(itemToSelectOnCancel);
                  ignoreTreeSelectionUpdate = false;
               });
            }
            return true;
         }
      }

      return false;
   }

   private void unloadEditor()
   {
      yoCompositePatternEditorPane.getChildren().clear();
      activeEditor.set(null);
   }

   private ObjectProperty<YoCompositePatternDefinition> newEmptyPattern()
   {
      if (shouldCancelAction(null))
         return null;
      else
         return new SimpleObjectProperty<>(new YoCompositePatternDefinition(NEW_PATTERN_NAME));
   }

   @FXML
   public void addPattern()
   {
      addPattern(new YoCompositePatternDefinition(NEW_PATTERN_NAME));
   }

   public void addPattern(YoCompositePatternDefinition pattern)
   {
      updateOrAddPattern(pattern, false);
   }

   public void updateOrAddPattern(YoCompositePattern pattern)
   {
      updateOrAddPattern(YoCompositeTools.toYoCompositePatternDefinition(pattern));
   }

   public void updateOrAddPattern(YoCompositePatternDefinition pattern)
   {
      updateOrAddPattern(pattern, true);
   }

   private void updateOrAddPattern(YoCompositePatternDefinition pattern, boolean allowUpdate)
   {
      if (shouldCancelAction(null))
         return;

      ObjectProperty<YoCompositePatternDefinition> patternProperty;

      if (allowUpdate)
      {
         Optional<ObjectProperty<YoCompositePatternDefinition>> matchingPattern = yoCompositePatternListView.getItems().stream()
                                                                                                            .filter(item -> item.get().getName()
                                                                                                                                .equals(pattern.getName()))
                                                                                                            .findFirst();
         if (matchingPattern.isPresent())
         {
            patternProperty = matchingPattern.get();
            if (patternProperty.isBound())
               cachedEditors.get(patternProperty).setInput(pattern);
            else
               patternProperty.set(pattern);
         }
         else
         {
            patternProperty = new SimpleObjectProperty<>(pattern);
            yoCompositePatternListView.getItems().add(patternProperty);
         }
      }
      else
      {
         patternProperty = new SimpleObjectProperty<>(pattern);
         yoCompositePatternListView.getItems().add(patternProperty);
      }
      yoCompositePatternListView.getSelectionModel().select(patternProperty);
   }

   @FXML
   public void removePattern()
   {
      if (shouldCancelAction(null))
         return;

      ObjectProperty<YoCompositePatternDefinition> selectedItem = yoCompositePatternListView.getSelectionModel().getSelectedItem();

      if (selectedItem == null)
         return;

      yoCompositePatternListView.getItems().remove(selectedItem);
      if (selectedItem.get() != null)
         yoCompositeSearchManager.discardYoComposite(selectedItem.get().getName());
   }

   public void removePattern(YoCompositePattern pattern)
   {
      removePattern(pattern.getType());
   }

   public void removePattern(YoCompositePatternDefinition pattern)
   {
      removePattern(pattern.getName());
   }

   public void removePattern(String patternName)
   {
      if (shouldCancelAction(null))
         return;

      Optional<ObjectProperty<YoCompositePatternDefinition>> firstPattern = yoCompositePatternListView.getItems().stream()
                                                                                                      .filter(p -> p.get().getName().equals(patternName))
                                                                                                      .findFirst();

      if (firstPattern.isPresent())
      {
         yoCompositePatternListView.getItems().remove(firstPattern.get());
         if (firstPattern.get().get() != null)
            yoCompositeSearchManager.discardYoComposite(firstPattern.get().get().getName());
      }
   }

   @FXML
   public void exportYoCompositePatterns()
   {
      File result = SessionVisualizerIOTools.yoCompositeConfigurationSaveFileDialog(window);
      if (result != null)
         messager.submitMessage(topics.getYoCompositePatternSaveRequest(), result);
   }

   @FXML
   public void importYoCompositePatterns()
   {
      if (shouldCancelAction(null))
         return;

      File result = SessionVisualizerIOTools.yoCompositeConfigurationOpenFileDialog(window);
      if (result != null)
         messager.submitMessage(topics.getYoCompositePatternLoadRequest(), result);
   }

   @FXML
   public void saveChanges()
   {
      YoCompositePatternEditorController editor = activeEditor.get();

      if (editor != null)
      {
         String originalDefinitionName = editor.getDefinitionBeforeEdits().getName();
         editor.saveChanges();
         yoCompositeSearchManager.discardYoComposite(originalDefinitionName);
         ObjectProperty<YoCompositePatternDefinition> patternDefinitionProperty = editor.patternDefinitionProperty();
         yoCompositeSearchManager.searchYoCompositeInBackground(YoCompositeTools.toYoCompositePattern(patternDefinitionProperty.get()));
      }
   }

   @FXML
   public void cancelChanges()
   {
      if (activeEditor.get() != null)
         activeEditor.get().resetFields();
   }

   public Stage getWindow()
   {
      return window;
   }
}
