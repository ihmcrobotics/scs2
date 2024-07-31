package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.MapChangeListener;
import javafx.collections.ObservableList;
import javafx.collections.ObservableMap;
import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.ComboBox;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TextField;
import javafx.scene.input.Clipboard;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DataFormat;
import javafx.scene.input.Dragboard;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.TransferMode;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeCollection;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

public class YoCompositeSearchPaneController extends ObservedAnimationTimer
{
   @FXML
   private TextField searchTextField;
   @FXML
   private ListView<YoComposite> yoCompositeListView;
   @FXML
   private ComboBox<String> searchTargetComboBox;

   private ObservableList<YoComposite> defaultItemList = FXCollections.emptyObservableList();
   private ObservableList<YoComposite> searchResult = null;

   private YoRegistry ownerRegistry = null;
   private YoCompositeSearchManager yoCompositeSearchManager;
   private BackgroundExecutorManager backgroundExecutorManager;

   private ObjectProperty<String> searchTargetProperty;

   private final Property<YoNameDisplay> yoVariableNameDisplay = new SimpleObjectProperty<>(this, "yoVariableNameDisplay", YoNameDisplay.SHORT_NAME);
   private AtomicReference<SearchEngines> activeSearchEngine;
   private AtomicReference<Integer> maxNumberOfItemsReference;
   private Future<ObservableList<YoComposite>> backgroundSearch;

   private Consumer<YoNamespace> registryViewRequestConsumer = null;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      initialize(toolkit, null);
   }

   public void initialize(SessionVisualizerToolkit toolkit, YoRegistry ownerRegistry)
   {
      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();
      YoManager yoManager = toolkit.getYoManager();
      backgroundExecutorManager = toolkit.getBackgroundExecutorManager();
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();

      Property<Integer> numberPrecision = messager.createPropertyInput(topics.getControlsNumberPrecision(), 3);

      Property<YoNameDisplay> yoVariableUserNameDisplay = new SimpleObjectProperty<>(this, "yoVariableUserNameDisplay", YoNameDisplay.SHORT_NAME);
      messager.bindBidirectional(topics.getYoVariableNameDisplay(), yoVariableUserNameDisplay, false);
      yoVariableUserNameDisplay.addListener((o, oldValue, newValue) -> search(searchTextField.getText()));

      this.ownerRegistry = ownerRegistry;

      if (ownerRegistry != null)
         yoVariableNameDisplay.setValue(YoNameDisplay.SHORT_NAME);
      else
         yoVariableNameDisplay.bindBidirectional(yoVariableUserNameDisplay);

      yoCompositeListView.setCellFactory(param -> new YoCompositeListCell(yoManager, yoVariableNameDisplay, numberPrecision, param));
      yoCompositeListView.getSelectionModel().setSelectionMode(SelectionMode.MULTIPLE);

      searchTextField.textProperty().addListener((observable, oldValue, newValue) -> search(newValue));

      activeSearchEngine = messager.createInput(topics.getYoSearchEngine(), SearchEngines.DEFAULT);
      maxNumberOfItemsReference = messager.createInput(topics.getYoSearchMaxListSize(), 500);

      Function<ListView<YoComposite>, MenuItem> openNamespace = listView ->
      {
         if (registryViewRequestConsumer == null)
            return null;
         YoComposite selectedItem = yoCompositeListView.getSelectionModel().getSelectedItem();
         if (selectedItem == null)
            return null;
         MenuItem menuItem = new MenuItem("Open namespace");
         YoNamespace namespace = selectedItem.getNamespace();
         menuItem.setOnAction(e -> registryViewRequestConsumer.accept(namespace));
         return menuItem;
      };
      Function<ListView<YoComposite>, MenuItem> copyVariableName = listView ->
      {
         YoComposite selectedItem = yoCompositeListView.getSelectionModel().getSelectedItem();
         if (selectedItem == null)
            return null;
         MenuItem menuItem = new MenuItem("Copy variable name");
         String name = selectedItem.getName();
         menuItem.setOnAction(e -> Clipboard.getSystemClipboard().setContent(Collections.singletonMap(DataFormat.PLAIN_TEXT, name)));
         return menuItem;
      };
      Function<ListView<YoComposite>, MenuItem> copyVariableFullname = listView ->
      {
         YoComposite selectedItem = yoCompositeListView.getSelectionModel().getSelectedItem();
         if (selectedItem == null)
            return null;
         MenuItem menuItem = new MenuItem("Copy variable fullname");
         String name = selectedItem.getFullname();
         menuItem.setOnAction(e -> Clipboard.getSystemClipboard().setContent(Collections.singletonMap(DataFormat.PLAIN_TEXT, name)));
         return menuItem;
      };
      Function<ListView<YoComposite>, MenuItem> showUniqueNames = listView ->
      {
         YoComposite selectedItem = yoCompositeListView.getSelectionModel().getSelectedItem();
         if (selectedItem == null)
            return null;
         CheckMenuItem menuItem = new CheckMenuItem("Show unique names");
         menuItem.setSelected(yoVariableNameDisplay.getValue() == YoNameDisplay.UNIQUE_SHORT_NAME);
         menuItem.setOnAction(e -> messager.submitMessage(topics.getYoVariableNameDisplay(),
                                                          menuItem.isSelected() ? YoNameDisplay.UNIQUE_SHORT_NAME : YoNameDisplay.SHORT_NAME));
         return menuItem;
      };
      MenuTools.setupContextMenu(yoCompositeListView, openNamespace, copyVariableName, copyVariableFullname, showUniqueNames);

      ObservableMap<String, Property<YoCompositeCollection>> nameToCompositeCollection = yoCompositeSearchManager.typeToCompositeCollection();
      searchTargetComboBox.setItems(FXCollections.observableArrayList(nameToCompositeCollection.keySet()));

      nameToCompositeCollection.addListener((MapChangeListener<String, Property<YoCompositeCollection>>) change ->
      {
         ObservableList<String> items = searchTargetComboBox.getItems();

         if (change.wasAdded())
         {
            if (!items.contains(change.getKey()))
               items.add(change.getKey());
         }
         else if (change.wasRemoved())
         {
            if (searchTargetComboBox.getValue() == change.getKey())
               searchTargetComboBox.getSelectionModel().select(YoCompositeTools.YO_VARIABLE);
            items.remove(change.getKey());
         }
      });

      searchTargetProperty = searchTargetComboBox.valueProperty();
      searchTargetProperty.set(YoCompositeTools.YO_VARIABLE);

      searchTargetProperty.addListener((o, oldValue, newValue) ->
                                       {
                                          if (newValue == null)
                                          {
                                             searchTargetProperty.set(oldValue);
                                             return;
                                          }
                                          else if (Objects.equals(newValue, oldValue) && !defaultItemList.isEmpty())
                                          {
                                             return;
                                          }

                                          refreshDefaultItemList(newValue);
                                          search(searchTextField.getText());
                                       });

      messager.addFXTopicListener(topics.getSessionCurrentState(), state ->
      {
         if (state == SessionState.ACTIVE)
            start();
         else if (state == SessionState.INACTIVE)
            stop();
      });

      yoCompositeListView.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
                                                                                 {
                                                                                    if (newValue != null)
                                                                                       messager.submitMessage(topics.getYoCompositeSelected(),
                                                                                                              Arrays.asList(newValue.getPattern().getType(),
                                                                                                                            newValue.getFullname()));
                                                                                 });
   }

   public void requestFocusForSearchBox()
   {
      searchTextField.requestFocus();
   }

   public void setRegistryViewRequestConsumer(Consumer<YoNamespace> consumer)
   {
      registryViewRequestConsumer = consumer;
   }

   public String getSearchTarget()
   {
      return searchTargetProperty.get();
   }

   @Override
   public void handleImpl(long now)
   {
      if (defaultItemList.isEmpty())
      {
         if (searchTargetProperty.get() == null)
         {
            searchTargetProperty.set(YoCompositeTools.YO_VARIABLE);
         }
         else
         {
            refreshDefaultItemList(searchTargetProperty.get());
            if (!searchTextField.getText().isEmpty())
               search(searchTextField.getText());
         }
      }

      if (backgroundSearch != null && backgroundSearch.isDone() && !backgroundSearch.isCancelled())
      {
         try
         {
            searchResult = backgroundSearch.get();
         }
         catch (InterruptedException | ExecutionException e)
         {
            e.printStackTrace();
         }
      }

      if (searchResult != null)
      {
         ObservableList<YoComposite> result = searchResult;
         searchResult = null;
         yoCompositeListView.setItems(result);
      }
   }

   @Override
   public void stop()
   {
      super.stop();
      defaultItemList = FXCollections.emptyObservableList();
      searchResult = null;
      yoCompositeListView.getItems().clear();
      searchTextField.clear();
      if (activeCompositeCollectionProperty != null)
         activeCompositeCollectionProperty.removeListener(autoRefreshListener);
      activeCompositeCollectionProperty = null;
   }

   private Property<YoCompositeCollection> activeCompositeCollectionProperty = null;
   private final ChangeListener<Object> autoRefreshListener = (o, oldV, newV) -> refreshDefaultItemList(getSearchTarget());

   private void refreshDefaultItemList(String searchTarget)
   {
      if (activeCompositeCollectionProperty != null)
         activeCompositeCollectionProperty.removeListener(autoRefreshListener);
      activeCompositeCollectionProperty = yoCompositeSearchManager.typeToCompositeCollection().get(searchTarget);
      activeCompositeCollectionProperty.addListener(autoRefreshListener);

      YoCompositeCollection yoVariableTypeReferenceCollection = activeCompositeCollectionProperty.getValue();
      if (yoVariableTypeReferenceCollection == null)
         defaultItemList = FXCollections.emptyObservableList();
      else if (ownerRegistry == null)
         defaultItemList = FXCollections.observableArrayList(yoVariableTypeReferenceCollection.getYoComposites());
      else
         defaultItemList = FXCollections.observableArrayList(yoVariableTypeReferenceCollection.getYoComposite(ownerRegistry));

      searchResult = defaultItemList;
   }

   private void search(String searchQuery)
   {
      if (backgroundSearch != null)
      {
         backgroundSearch.cancel(true);
         backgroundSearch = null;
      }

      if (searchQuery != null && !searchQuery.isEmpty() && defaultItemList != null && !defaultItemList.isEmpty())
      {
         backgroundSearch = backgroundExecutorManager.executeInBackground(() -> performSearch(searchQuery));
      }
      else
      {
         searchResult = defaultItemList;
      }
   }

   private ObservableList<YoComposite> performSearch(String searchQuery)
   {
      Function<YoComposite, String> nameExtractor = switch (yoVariableNameDisplay.getValue())
      {
         case UNIQUE_NAME -> YoComposite::getUniqueName;
         case UNIQUE_SHORT_NAME -> YoComposite::getUniqueShortName;
         case SHORT_NAME -> YoComposite::getName;
         case FULL_NAME -> YoComposite::getFullname;
      };

      List<YoComposite> yoVariables = YoVariableTools.search(defaultItemList,
                                                             nameExtractor,
                                                             searchQuery,
                                                             YoVariableTools.fromSearchEnginesEnum(activeSearchEngine.get()),
                                                             maxNumberOfItemsReference.get());
      return yoVariables == null ? FXCollections.emptyObservableList() : FXCollections.observableArrayList(yoVariables);
   }

   @FXML
   public void startYoVariableDragAndDrop(MouseEvent event)
   {
      if (!event.isPrimaryButtonDown())
         return;

      List<YoComposite> yoComposites = yoCompositeListView.getSelectionModel().getSelectedItems();

      Dragboard dragBoard = yoCompositeListView.startDragAndDrop(TransferMode.COPY);
      ClipboardContent clipboardContent = new ClipboardContent();

      if (yoComposites.size() == 1)
      {
         YoComposite yoComposite = yoComposites.get(0);
         if (yoComposite == null)
            return;
         clipboardContent.put(DragAndDropTools.YO_COMPOSITE_REFERENCE, Arrays.asList(yoComposite.getPattern().getType(), yoComposite.getFullname()));
      }
      else
      {
         List<String> content = yoComposites.stream()
                                            .flatMap(c -> Arrays.asList(c.getPattern().getType(), c.getFullname()).stream())
                                            .collect(Collectors.toList());
         clipboardContent.put(DragAndDropTools.YO_COMPOSITE_LIST_REFERENCE, content);
      }
      dragBoard.setContent(clipboardContent);
      event.consume();
   }
}
