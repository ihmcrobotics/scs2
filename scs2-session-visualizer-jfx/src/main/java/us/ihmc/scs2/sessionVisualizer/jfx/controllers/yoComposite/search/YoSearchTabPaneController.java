package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TabPane.TabClosingPolicy;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoSearchTabPaneController
{
   @FXML
   private TabPane yoSearchTabPane;
   @FXML
   private YoRegistrySearchPaneController yoRegistrySearchPaneController;
   @FXML
   private YoCompositeSearchPaneController mainYoCompositeSearchPaneController;

   private SessionVisualizerToolkit toolkit;
   private final Map<YoRegistry, Tab> registryTabs = new HashMap<>();
   private final Map<Tab, YoCompositeSearchPaneController> tabCompositeControllerMap = new HashMap<>();

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      yoSearchTabPane.setTabClosingPolicy(TabClosingPolicy.ALL_TABS);
      yoSearchTabPane.getSelectionModel().select(1);
      tabCompositeControllerMap.put(yoSearchTabPane.getSelectionModel().getSelectedItem(), mainYoCompositeSearchPaneController);
      yoRegistrySearchPaneController.initialize(toolkit);
      yoRegistrySearchPaneController.setRegistryViewRequestConsumer(newRequest ->
      {
         if (newRequest != null)
            openRegistryTab(newRequest, 1);
      });

      mainYoCompositeSearchPaneController.initialize(toolkit);
      mainYoCompositeSearchPaneController.setRegistryViewRequestConsumer(newRequest ->
      {
         if (newRequest == null)
            return;
         try
         {
            YoRegistry registry = toolkit.getYoManager().getRootRegistry().findRegistry(newRequest);
            openRegistryTab(registry, -1);
         }
         catch (RuntimeException e)
         {
            LogTools.error("Registry not found: " + newRequest);
         }
      });

      // By disabling the search tabs, we unlink YoVariables reducing the cost of a run tick for the Session
      yoSearchTabPane.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
      {
         for (Tab tab : yoSearchTabPane.getTabs())
         {
            Node content = tab.getContent();
            if (content != null)
               content.setDisable(tab != newValue);
         }

         JavaFXMissingTools.runLater(getClass(), this::requestFocusForActiveSearchBox);
      });
   }

   public void requestFocusForActiveSearchBox()
   {
      YoCompositeSearchPaneController controller = tabCompositeControllerMap.get(yoSearchTabPane.getSelectionModel().getSelectedItem());
      if (controller != null)
      {
         controller.requestFocusForSearchBox();
      }
      else
      {
         yoRegistrySearchPaneController.requestFocusForSearchBox();
      }
   }

   public void start()
   {
      yoRegistrySearchPaneController.start();
      mainYoCompositeSearchPaneController.start();
   }

   public void stop()
   {
      yoRegistrySearchPaneController.stop();
      mainYoCompositeSearchPaneController.stop();
   }

   private void openRegistryTab(YoRegistry registry, int tabIndex)
   {
      if (registryTabs.containsKey(registry))
      {
         yoSearchTabPane.getSelectionModel().select(registryTabs.get(registry));
      }
      else
      {
         Tab newTab = new Tab(registry.getName());
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_COMPOSITE_SEARCH_PANEL_URL);
         try
         {
            Pane rootPane = loader.load();
            YoCompositeSearchPaneController controller = loader.getController();
            controller.initialize(toolkit, registry);
            controller.start();
            registryTabs.put(registry, newTab);
            newTab.setContent(rootPane);
            tabCompositeControllerMap.put(newTab, controller);
            newTab.setClosable(true);
            newTab.setOnClosed(e ->
            {
               controller.stop();
               registryTabs.remove(registry);
               tabCompositeControllerMap.remove(newTab);
            });
            if (tabIndex == -1)
               yoSearchTabPane.getTabs().add(yoSearchTabPane.getTabs().size() - 1, newTab);
            else
               yoSearchTabPane.getTabs().add(tabIndex, newTab);
            yoSearchTabPane.getSelectionModel().select(newTab);
         }
         catch (IOException e)
         {
            throw new RuntimeException("Could not load the registry tab.", e);
         }
      }
   }

   @FXML
   void startYoVariableDragAndDrop(MouseEvent event)
   {
      mainYoCompositeSearchPaneController.startYoVariableDragAndDrop(event);
   }
}
