package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor;

import static us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools.addAfterMenuItemFactory;
import static us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools.addBeforeMenuItemFactory;
import static us.ihmc.scs2.sessionVisualizer.jfx.tools.ListViewTools.removeMenuItemFactory;

import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

import javafx.animation.AnimationTimer;
import javafx.beans.binding.BooleanBinding;
import javafx.beans.binding.BooleanExpression;
import javafx.beans.binding.NumberBinding;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Button;
import javafx.scene.control.ContentDisplay;
import javafx.scene.control.Label;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DataFormat;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.Region;
import javafx.scene.layout.VBox;
import us.ihmc.scs2.definition.yoComposite.YoCompositeDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXControllerTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoCompositeListTextField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoIntegerTextField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.editor.yoTextField.YoReferenceFrameTextField;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.CompositeProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

public class YoCompositeListEditorPaneController
{
   // FIXME Drag-n-drop for re-ordering list cells isn't working properly
   @FXML
   private VBox mainPane;
   @FXML
   private GridPane listSearchGridPane;
   @FXML
   private Label compositeListLabel, listFrameLabel;
   @FXML
   private TextField compositeListSearchTextField, referenceFrameSearchTextField;
   @FXML
   private Button addCompositeButton;
   @FXML
   private ListView<YoCompositeEditorPaneController> listView;
   @FXML
   private Label numberOfCompositesLabel;
   @FXML
   private TextField numberOfCompositesTextField;
   @FXML
   private ImageView numberOfCompositesValidImageView;

   private final StringProperty compositeNameProperty = new SimpleStringProperty(this, "compositeName", "composite");
   private final StringProperty compositesNameProperty = new SimpleStringProperty(this, "compositesName", "composites");

   private ObservableBooleanValue inputsValidityProperty;
   private BooleanProperty compositesValidityProperty = new SimpleBooleanProperty(this, "compositesValidity", false);
   private YoReferenceFrameTextField yoReferenceFrameTextField;
   private YoIntegerTextField yoNumberOfCompositesTextField;

   private final ObjectProperty<IntegerProperty> numberOfCompositesProperty = new SimpleObjectProperty<>(this, "numberOfComposites", null);
   private final ObjectProperty<List<DoubleProperty[]>> compositeListProperty = new SimpleObjectProperty<>(this, "compositeList", null);

   private SessionVisualizerToolkit toolkit;

   private URL yoCompositeEditorPaneFXML = SessionVisualizerIOTools.YO_COMPOSITE_EDITOR_URL;
   private YoCompositeCollection yoCompositeCollection;
   private boolean setupReferenceFrameFields;

   public void initialize(SessionVisualizerToolkit toolkit, YoCompositeCollection yoCompositeCollection, boolean setupReferenceFrameFields)
   {
      this.toolkit = toolkit;
      this.yoCompositeCollection = yoCompositeCollection;
      this.setupReferenceFrameFields = setupReferenceFrameFields;

      YoCompositeSearchManager yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();
      LinkedYoRegistry linkedRootRegistry = toolkit.getYoManager().getLinkedRootRegistry();
      yoNumberOfCompositesTextField = new YoIntegerTextField(numberOfCompositesTextField,
                                                             yoCompositeSearchManager,
                                                             linkedRootRegistry,
                                                             true,
                                                             numberOfCompositesValidImageView);
      setupListViewControls();

      if (yoCompositeCollection != null)
      {
         yoCompositeSearchManager.requestSearchListOfYoComposites(yoCompositeCollection.getPattern(),
                                                                  (Consumer<Map<String, List<YoComposite>>>) compositeListMap ->
                                                                  {
                                                                     YoCompositeListTextField yoCompositeListTextField = new YoCompositeListTextField(compositeListMap,
                                                                                                                                                      compositeListSearchTextField);
                                                                     yoCompositeListTextField.setupAutoCompletion();
                                                                     yoCompositeListTextField.compositeListProperty()
                                                                                             .addListener((o, oldValue, newValue) -> setComposites(newValue));
                                                                  });
      }

      if (!setupReferenceFrameFields)
      {
         listSearchGridPane.getChildren().removeAll(listFrameLabel, referenceFrameSearchTextField);
         listSearchGridPane.getRowConstraints().remove(1);
      }
      else
      {
         ReferenceFrameManager referenceFrameManager = toolkit.getReferenceFrameManager();
         yoReferenceFrameTextField = new YoReferenceFrameTextField(referenceFrameSearchTextField, referenceFrameManager);
         yoReferenceFrameTextField.setupAutoCompletion();
         yoReferenceFrameTextField.supplierProperty().addListener((o, oldValue, newValue) ->
         {
            if (newValue == null)
               return;

            listView.getItems().forEach(controller -> controller.setReferenceFrame(newValue.getValue()));
         });
      }

      yoNumberOfCompositesTextField.setupAutoCompletion();

      compositesNameProperty.addListener((observable, oldValue, newValue) ->
      {
         if (newValue == null || newValue.isEmpty())
         {
            compositesNameProperty.set(oldValue);
            return;
         }
         numberOfCompositesLabel.setText(YoGraphicFXControllerTools.replaceAndMatchCase(numberOfCompositesLabel.getText(), oldValue, newValue));
      });
      compositeNameProperty.addListener((observable, oldValue, newValue) ->
      {
         if (newValue == null || newValue.isEmpty())
         {
            compositeNameProperty.set(oldValue);
            return;
         }
         compositeListLabel.setText(YoGraphicFXControllerTools.replaceAndMatchCase(compositeListLabel.getText(), oldValue, newValue));
      });

      inputsValidityProperty = compositesValidityProperty.and(yoNumberOfCompositesTextField.getValidityProperty());

      yoNumberOfCompositesTextField.supplierProperty().addListener((o, oldValue, newValue) -> numberOfCompositesProperty.set(newValue));
   }

   private void setupListViewControls()
   {
      listView.setCellFactory(param -> new CompositeEditorListCell());
      listView.getSelectionModel().setSelectionMode(SelectionMode.MULTIPLE);

      listView.getItems().addListener((ListChangeListener<YoCompositeEditorPaneController>) change ->
      {
         if (change.getList().isEmpty())
         {
            compositesValidityProperty.unbind();
            compositesValidityProperty.set(false);
         }
         else
         {
            BooleanExpression observable = BooleanBinding.booleanExpression(change.getList().get(0).inputsValidityProperty());
            for (int i = 1; i < change.getList().size(); i++)
               observable = observable.and(change.getList().get(i).inputsValidityProperty());
            compositesValidityProperty.bind(observable);
         }
      });

      listView.setOnDragDetected(this::handleDragDetected);
      MenuTools.setupContextMenu(listView,
                                        addBeforeMenuItemFactory(this::newYoCompositeEditor),
                                        addAfterMenuItemFactory(this::newYoCompositeEditor),
                                        removeMenuItemFactory(false));

      listView.getItems().addListener((ListChangeListener<YoCompositeEditorPaneController>) change ->
      {
         List<DoubleProperty[]> newCompositeList = new ArrayList<>();

         for (YoCompositeEditorPaneController item : change.getList())
         {
            int compositeIndex = newCompositeList.size();
            ReadOnlyObjectProperty<DoubleProperty[]> compositeSupplierProperty = item.compositeSupplierProperty();
            newCompositeList.add(compositeSupplierProperty.get());

            compositeSupplierProperty.addListener((o, oldValue, newValue) ->
            {
               List<DoubleProperty[]> listUpdated = new ArrayList<>(compositeListProperty.get());
               listUpdated.set(compositeIndex, newValue);
               compositeListProperty.set(listUpdated);
            });
         }

         compositeListProperty.set(newCompositeList);
      });
   }

   @FXML
   public void addComposite()
   {
      YoCompositeEditorPaneController newCompositeEditor = newYoCompositeEditor();
      newCompositeEditor.setCompositeName(compositeNameProperty.get());
      if (setupReferenceFrameFields && yoReferenceFrameTextField.getSupplier() != null)
         newCompositeEditor.setReferenceFrame(yoReferenceFrameTextField.getSupplier().getValue());
      listView.getItems().add(newCompositeEditor);
      // FIXME This doesn't seem reliable, also should force the ListView to scroll down the item is guaranteed to be visible.
      JavaFXMissingTools.runLater(getClass(), () -> newCompositeEditor.getSearchYoCompositeTextField().requestFocus());
   }

   private YoCompositeEditorPaneController newYoCompositeEditor()
   {
      FXMLLoader loader = new FXMLLoader(yoCompositeEditorPaneFXML);
      try
      {
         loader.load();
         YoCompositeEditorPaneController editor = loader.getController();
         editor.initialize(toolkit, yoCompositeCollection, setupReferenceFrameFields);
         return editor;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private void setComposites(List<YoComposite> yoCompositeList)
   {
      ObservableList<YoCompositeEditorPaneController> listViewItems = listView.getItems();

      while (listViewItems.size() < yoCompositeList.size())
         listViewItems.add(newYoCompositeEditor());

      while (listViewItems.size() > yoCompositeList.size())
         listViewItems.remove(listViewItems.size() - 1);

      for (int i = 0; i < listViewItems.size(); i++)
      {
         YoComposite yoComposite = yoCompositeList.get(i);
         listViewItems.get(i).setInput(yoComposite);
      }
   }

   public void setInputFromDefinition(List<? extends YoCompositeDefinition> input, String numberOfComposites)
   {
      ObservableList<YoCompositeEditorPaneController> listViewItems = listView.getItems();

      while (listViewItems.size() < input.size())
         listViewItems.add(newYoCompositeEditor());

      while (listViewItems.size() > input.size())
         listViewItems.remove(listViewItems.size() - 1);

      for (int i = 0; i < listViewItems.size(); i++)
      {
         listViewItems.get(i).setInput(input.get(i));
      }

      numberOfCompositesTextField.setText(numberOfComposites);
   }

   public void setInputSingletonComposites(YoListDefinition input)
   {
      ObservableList<YoCompositeEditorPaneController> listViewItems = listView.getItems();

      if (input.getElements() == null)
      {
         listViewItems.clear();
      }
      else
      {
         while (listViewItems.size() < input.getElements().size())
            listViewItems.add(newYoCompositeEditor());

         while (listViewItems.size() > input.getElements().size())
            listViewItems.remove(listViewItems.size() - 1);

         for (int i = 0; i < listViewItems.size(); i++)
         {
            listViewItems.get(i).setInput(input.getElements().get(i));
         }
      }

      numberOfCompositesTextField.setText(input.getSize());
   }

   public void setInputSingletonComposites(List<String> input, String numberOfComposites)
   {
      ObservableList<YoCompositeEditorPaneController> listViewItems = listView.getItems();

      if (input == null)
      {
         listViewItems.clear();
      }
      else
      {
         while (listViewItems.size() < input.size())
            listViewItems.add(newYoCompositeEditor());

         while (listViewItems.size() > input.size())
            listViewItems.remove(listViewItems.size() - 1);

         for (int i = 0; i < listViewItems.size(); i++)
         {
            listViewItems.get(i).setInput(input.get(i));
         }
      }

      numberOfCompositesTextField.setText(numberOfComposites);
   }

   public void setCompositeName(String compositeName)
   {
      setCompositeName(compositeName, null);
   }

   public void setCompositeName(String compositeName, String compositesName)
   {
      this.compositeNameProperty.set(compositeName);
      this.compositesNameProperty.set(compositesName == null ? compositeName + "s" : compositesName);
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public ReadOnlyObjectProperty<IntegerProperty> numberOfCompositesProperty()
   {
      return numberOfCompositesProperty;
   }

   public ReadOnlyObjectProperty<List<DoubleProperty[]>> compositeListProperty()
   {
      return compositeListProperty;
   }

   public void addInputNotification(Runnable callback)
   {
      listView.getItems().addListener((ListChangeListener<YoCompositeEditorPaneController>) change ->
      {
         change.getList().forEach(controller -> controller.addInputNotification(callback));
         callback.run();
      });
      numberOfCompositesProperty.addListener((o, oldValue, newValue) -> callback.run());
   }

   public <T extends CompositeProperty> void addInputListener(Consumer<List<T>> listConsumer, Supplier<T> compositeBuilder)
   {
      listView.getItems().addListener((ListChangeListener<YoCompositeEditorPaneController>) change ->
      {
         ObservableList<? extends YoCompositeEditorPaneController> newList = change.getList();
         List<T> compositePropertyList = new ArrayList<>(newList.size());

         for (YoCompositeEditorPaneController controller : newList)
         {
            T composite = compositeBuilder.get();
            if (setupReferenceFrameFields)
               composite.set(controller.frameSupplierProperty().getValue(), controller.compositeSupplierProperty().get());
            else
               composite.setComponentValueProperties(controller.compositeSupplierProperty().get());
            controller.bindYoCompositeDoubleProperty(composite);
            compositePropertyList.add(composite);
         }

         listConsumer.accept(compositePropertyList);
      });
   }

   public Pane getMainPane()
   {
      return mainPane;
   }

   private AnimationTimer scrollPaneHeightAdjustmentAnimation;
   private ObjectProperty<NumberBinding> extraFieldsHeightProperty;

   public void setupHeightAdjustmentForScrollPane(ScrollPane scrollPane)
   {
      if (nCellsHeightAdjustmentAnimation != null)
         nCellsHeightAdjustmentAnimation.stop();

      NumberBinding extraFieldsHeight = ((Region) scrollPane.contentProperty().get()).heightProperty().subtract(listView.heightProperty());

      if (extraFieldsHeightProperty == null)
         extraFieldsHeightProperty = new SimpleObjectProperty<NumberBinding>(this, "extraFieldsHeight", null);
      extraFieldsHeightProperty.set(extraFieldsHeight);

      if (scrollPaneHeightAdjustmentAnimation == null)
      {
         scrollPaneHeightAdjustmentAnimation = new ObservedAnimationTimer(getClass().getSimpleName())
         {
            @Override
            public void handleImpl(long now)
            {
               if (extraFieldsHeightProperty.get() == null)
                  return;

               double containerHeight = scrollPane.getHeight();
               double prefHeight = containerHeight - extraFieldsHeightProperty.get().doubleValue() - 5.0;

               if (prefHeight > 0)
                  listView.setPrefHeight(prefHeight);
               else
                  listView.setPrefHeight(Region.USE_COMPUTED_SIZE);

               if (listView.getItems().isEmpty())
               {
                  listView.setMinHeight(0.0);
                  listView.setMaxHeight(0.0);
               }
               else
               {
                  double minHeight = listView.getItems().get(0).getMainPane().getHeight() + 10.0;
                  listView.setMinHeight(minHeight);
                  listView.setMaxHeight(listView.getItems().size() * minHeight);
               }
            }
         };
      }

      scrollPaneHeightAdjustmentAnimation.start();
   }

   private AnimationTimer nCellsHeightAdjustmentAnimation;
   private IntegerProperty numberOfCellsToViewProperty;

   public void setPrefHeight(int numberOfCellsToView)
   {
      if (scrollPaneHeightAdjustmentAnimation != null)
         scrollPaneHeightAdjustmentAnimation.stop();

      if (numberOfCellsToViewProperty == null)
         numberOfCellsToViewProperty = new SimpleIntegerProperty(this, "numberOfCellsToView", 0);

      numberOfCellsToViewProperty.set(numberOfCellsToView);

      if (nCellsHeightAdjustmentAnimation == null)
      {
         nCellsHeightAdjustmentAnimation = new ObservedAnimationTimer(getClass().getSimpleName())
         {
            @Override
            public void handleImpl(long now)
            {
               if (listView.getItems().isEmpty())
               {
                  listView.setMinHeight(0.0);
                  listView.setMaxHeight(0.0);
                  listView.setPrefHeight(0.0);
               }
               else
               {
                  double minHeight = listView.getItems().get(0).getMainPane().getHeight() + 10.0;
                  listView.setMinHeight(minHeight);
                  listView.setPrefHeight(numberOfCellsToViewProperty.get() * minHeight);
                  listView.setMaxHeight(2.0 * numberOfCellsToViewProperty.get() * minHeight);
               }
            }
         };
      }

      nCellsHeightAdjustmentAnimation.start();
   }

   private class CompositeEditorListCell extends ListCell<YoCompositeEditorPaneController>
   {
      public CompositeEditorListCell()
      {
         setOnDragOver(this::handleDragOver);
         setOnDragEntered(this::handleDragEntered);
         setOnDragExited(this::handleDragExited);
         setOnDragDropped(this::handleDragDropped);
         setOnDragDone(this::handleDragDone);
      }

      @Override
      protected void updateItem(YoCompositeEditorPaneController item, boolean empty)
      {
         super.updateItem(item, empty);
         setText(null);

         if (empty)
         {
            setGraphic(null);
         }
         else
         {
            item.setCompositeName(compositeNameProperty.get() + " " + getIndex());
            Pane mainPane = item.getMainPane();
            setGraphic(mainPane);
            setContentDisplay(ContentDisplay.BOTTOM);
            // Somehow the width of the cell is too large...
            mainPane.prefWidthProperty().bind(widthProperty().subtract(20));
         }
      }

      public void handleDragOver(DragEvent event)
      {
         if (!event.isAccepted() && acceptDragEventForDrop(event))
            event.acceptTransferModes(TransferMode.ANY);
         event.consume();
      }

      private void handleDragEntered(DragEvent event)
      {
         if (!event.isAccepted() && acceptDragEventForDrop(event))
            setSelectionHighlight(true);
         event.consume();
      }

      private void handleDragExited(DragEvent event)
      {
         if (acceptDragEventForDrop(event))
            setSelectionHighlight(false);
         event.consume();
      }

      private void setSelectionHighlight(boolean isSelected)
      {
         if (isSelected)
            setStyle("-fx-border-color:green; -fx-border-radius:5;");
         else
            setStyle("-fx-border-color: null;");
      }

      private void handleDragDropped(DragEvent event)
      {
         Dragboard dragboard = event.getDragboard();
         boolean success = false;

         if (dragboard.hasContent(YO_COMPOSITE_CELL))
         {
            ObservableList<YoCompositeEditorPaneController> vertexListViewItems = listView.getItems();

            if (itemsToMove != null && !itemsToMove.isEmpty())
            {
               int dropIndex = getIndex();
               if (dropIndex < vertexListViewItems.size())
                  vertexListViewItems.addAll(dropIndex, itemsToMove);
               else
                  vertexListViewItems.addAll(itemsToMove);

               setSelectionHighlight(false);
               success = true;
            }
         }
         event.setDropCompleted(success);
         event.consume();
      }

      private void handleDragDone(DragEvent event)
      {
         if (!event.isDropCompleted() && itemsOriginal != null)
         {
            listView.getItems().setAll(itemsOriginal);
         }

         itemsOriginal = null;
         itemsToMove = null;
         event.consume();
      }

      private boolean acceptDragEventForDrop(DragEvent event)
      {
         if (!event.getDragboard().hasContent(YO_COMPOSITE_CELL))
            return false;
         return !listView.getSelectionModel().getSelectedItems().contains(getItem());
      }
   }

   private static final DataFormat YO_COMPOSITE_CELL = new DataFormat("yoCompositeListViewController/compositeCell");
   private List<YoCompositeEditorPaneController> itemsToMove = null;
   private List<YoCompositeEditorPaneController> itemsOriginal = null;

   private void handleDragDetected(MouseEvent event)
   {
      ObservableList<YoCompositeEditorPaneController> selectedItems = listView.getSelectionModel().getSelectedItems();

      if (listView.getItems().size() > selectedItems.size())
      {
         Dragboard dragBoard = listView.startDragAndDrop(TransferMode.ANY);
         ClipboardContent clipboardContent = new ClipboardContent();
         clipboardContent.put(YO_COMPOSITE_CELL, "Dummy");
         dragBoard.setContent(clipboardContent);

         itemsToMove = new ArrayList<>(selectedItems);
         itemsOriginal = new ArrayList<>(listView.getItems());
         listView.getItems().removeAll(itemsToMove);
         listView.getSelectionModel().clearSelection();

         AnchorPane[] panes = itemsToMove.stream().map(YoCompositeEditorPaneController::getMainPane).toArray(AnchorPane[]::new);
         int height = (int) Stream.of(panes).mapToDouble(AnchorPane::getHeight).sum();
         int width = (int) Stream.of(panes).mapToDouble(AnchorPane::getWidth).max().getAsDouble();
         VBox vBox = new VBox(panes);
         vBox.resize(width, height);
         WritableImage image = new WritableImage(width, height);
         vBox.snapshot(null, image);
         dragBoard.setDragView(image);
      }

      event.consume();
   }
}
