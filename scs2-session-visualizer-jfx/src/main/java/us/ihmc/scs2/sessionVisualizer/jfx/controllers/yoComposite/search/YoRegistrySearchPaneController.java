package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import javafx.beans.property.Property;
import javafx.fxml.FXML;
import javafx.scene.control.MenuItem;
import javafx.scene.control.TextField;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeView;
import javafx.scene.input.*;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager.NewWindowRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TreeViewTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.YoVariableTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.stream.Collectors;

public class YoRegistrySearchPaneController extends ObservedAnimationTimer
{
   @FXML
   private TextField searchTextField;

   @FXML
   private TreeView<YoRegistry> registryTreeView;

   private List<YoRegistry> allRegistries;

   private TreeItem<YoRegistry> defaultRootItem;
   private TreeItem<YoRegistry> searchResult = null;
   private boolean showRoot = true;

   private Property<Boolean> showSCS2YoVariables;
   private Predicate<YoRegistry> scs2InternalRegistryFilter;

   private AtomicReference<SearchEngines> activeSearchEngine;

   private YoRegistry rootRegistry;

   private Future<TreeItem<YoRegistry>> backgroundSearch;

   private boolean refreshRootRegistry;

   private YoManager yoManager;
   private BackgroundExecutorManager backgroundExecutorManager;

   private Consumer<YoRegistry> registryViewRequestConsumer = null;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      yoManager = toolkit.getYoManager();
      backgroundExecutorManager = toolkit.getBackgroundExecutorManager();
      registryTreeView.setCellFactory(param -> new YoRegistryTreeCell());
      registryTreeView.setRoot(defaultRootItem);
      JavaFXMessager messager = toolkit.getMessager();
      SessionVisualizerTopics topics = toolkit.getTopics();

      MenuTools.setupContextMenu(registryTreeView, treeView ->
      {
         MenuItem openStatisticsMenuItem = new MenuItem("Open statistics...");
         openStatisticsMenuItem.setOnAction(e ->
                                            {
                                               TreeItem<YoRegistry> selectedRegistry = treeView.getSelectionModel().getSelectedItem();
                                               if (selectedRegistry == null)
                                                  return;
                                               messager.submitMessage(topics.getOpenWindowRequest(),
                                                                      NewWindowRequest.registryStatisticWindow(toolkit.getMainWindow(),
                                                                                                               selectedRegistry.getValue()));
                                            });
         return openStatisticsMenuItem;
      });
      yoManager.rootRegistryChangeCounter().addListener((o, oldValue, newValue) -> refreshRootRegistry = true);

      searchTextField.textProperty().addListener((observable, oldValue, newValue) -> search(newValue));

      activeSearchEngine = messager.createInput(topics.getYoSearchEngine(), SearchEngines.DEFAULT);

      messager.addFXTopicListener(topics.getSessionCurrentState(), state ->
      {
         if (state == SessionState.ACTIVE)
         {
            refreshRootRegistry = true;
            start();
         }
         else if (state == SessionState.INACTIVE)
         {
            stop();
            searchResult = null;
            allRegistries = Collections.emptyList();
            defaultRootItem = new TreeItem<>();
            registryTreeView.setRoot(defaultRootItem);
            searchTextField.clear();
         }
      });

      showSCS2YoVariables = messager.createPropertyInput(topics.getShowSCS2YoVariables(), false);
      showSCS2YoVariables.addListener((o, oldValue, newValue) -> refreshRootRegistry = true);
      scs2InternalRegistryFilter = reg -> showSCS2YoVariables.getValue() || !reg.getNamespace().equals(Session.SESSION_INTERNAL_NAMESPACE);
   }

   public void requestFocusForSearchBox()
   {
      searchTextField.requestFocus();
   }

   public void setRegistryViewRequestConsumer(Consumer<YoRegistry> consumer)
   {
      this.registryViewRequestConsumer = consumer;
   }

   private void refreshRootRegistry()
   {
      rootRegistry = yoManager.getRootRegistry();

      if (rootRegistry == null)
      {
         refreshRootRegistry = true;
         allRegistries = null;
         defaultRootItem = null;
      }
      else
      {
         allRegistries = rootRegistry.collectSubtreeRegistries();
         defaultRootItem = new TreeItem<>(rootRegistry);
         buildTreeRecursively(defaultRootItem, scs2InternalRegistryFilter);
      }
   }

   @Override
   public void handleImpl(long now)
   {
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
         registryTreeView.setShowRoot(showRoot);
         registryTreeView.setRoot(searchResult);
         searchResult = null;
      }

      if (refreshRootRegistry)
      {
         refreshRootRegistry = false;
         refreshRootRegistry();
         if (!refreshRootRegistry)
            search(searchTextField.getText());
      }
   }

   private void search(String searchQuery)
   {
      if (backgroundSearch != null)
      {
         backgroundSearch.cancel(true);
         backgroundSearch = null;
      }

      if (searchQuery != null && !searchQuery.isEmpty())
      {
         backgroundSearch = backgroundExecutorManager.executeInBackground(() ->
                                                                          {
                                                                             if (allRegistries == null)
                                                                             {
                                                                                refreshRootRegistry = true;
                                                                                return null;
                                                                             }

                                                                             return createRootItemForRegistries(YoVariableTools.search(allRegistries,
                                                                                                                                       registry -> registry.getNamespace()
                                                                                                                                                           .getName(),
                                                                                                                                       searchQuery,
                                                                                                                                       YoVariableTools.fromSearchEnginesEnum(
                                                                                                                                             activeSearchEngine.get()),
                                                                                                                                       Integer.MAX_VALUE,
                                                                                                                                       Collectors.toSet()));
                                                                          });
      }
      else
      {
         searchResult = defaultRootItem;
         showRoot = false;
      }
   }

   @FXML
   void openRegistryTab(InputEvent inputEvent)
   {
      if (registryViewRequestConsumer == null)
         return;

      boolean performOpenAction = false;

      if (inputEvent instanceof MouseEvent)
      {
         MouseEvent mouseEvent = (MouseEvent) inputEvent;
         performOpenAction = (mouseEvent.getButton() == MouseButton.PRIMARY && mouseEvent.getClickCount() == 2);
      }
      else if (inputEvent instanceof KeyEvent)
      {
         KeyEvent keyEvent = (KeyEvent) inputEvent;
         performOpenAction = keyEvent.getCode() == KeyCode.ENTER;
      }

      if (performOpenAction)
      {
         TreeItem<YoRegistry> selectedItem = registryTreeView.getSelectionModel().getSelectedItem();

         if (selectedItem != null)
         {
            registryViewRequestConsumer.accept(selectedItem.getValue());
            inputEvent.consume();
         }
      }
   }

   private TreeItem<YoRegistry> createRootItemForRegistries(Set<YoRegistry> subSelection)
   {
      TreeItem<YoRegistry> root = new TreeItem<>(rootRegistry);
      buildTreeRecursively(root, scs2InternalRegistryFilter);
      filterRegistries(root, subSelection);
      TreeViewTools.expandRecursively(root);
      return root;
   }

   private static void filterRegistries(TreeItem<YoRegistry> parent, Set<YoRegistry> registriesToKeep)
   {
      if (parent == null || parent.isLeaf())
         return;

      for (TreeItem<YoRegistry> child : parent.getChildren())
      {
         if (!child.getChildren().isEmpty())
            filterRegistries(child, registriesToKeep);
      }

      for (int i = parent.getChildren().size() - 1; i >= 0; i--)
      {
         TreeItem<YoRegistry> child = parent.getChildren().get(i);

         if (!child.isLeaf() || child.getValue() == null)
            continue;

         if (!registriesToKeep.contains(child.getValue()))
            parent.getChildren().remove(i);
      }
   }

   private static void buildTreeRecursively(TreeItem<YoRegistry> parent, Predicate<YoRegistry> registryFilter)
   {
      List<YoRegistry> children = parent.getValue().getChildren();

      for (int i = 0; i < children.size(); i++)
      {
         YoRegistry child = children.get(i);

         if (!registryFilter.test(child))
            continue;

         TreeItem<YoRegistry> childItem = new TreeItem<>(child);
         parent.getChildren().add(childItem);
         buildTreeRecursively(childItem, registryFilter);
      }
   }
}
