package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import javax.xml.bind.JAXBException;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.stage.Stage;
import javafx.stage.StageStyle;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TabPaneTools;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;

public class YoMultiBCF2000SliderboardWindowController
{
   @FXML
   private TabPane sliderboardTabPane;
   @FXML
   private Tab initialTab;
   @FXML
   private YoBCF2000SliderboardWindowController initialSliderboardPaneController;

   private final Map<Tab, YoBCF2000SliderboardWindowController> tabToControllerMap = new HashMap<>();

   private SessionVisualizerToolkit toolkit;

   private Stage window;
   private Window owner;

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      owner = toolkit.getMainWindow();
      window = new Stage(StageStyle.UTILITY);
      initialSliderboardPaneController.initialize(window, toolkit);
      Label initialTabHeader = TabPaneTools.editableTabHeader(initialTab);

      tabToControllerMap.put(initialTab, initialSliderboardPaneController);
      MenuTools.setupContextMenu(sliderboardTabPane,
                                 TabPaneTools.addBeforeMenuItemFactory(this::newSliderboardTab),
                                 TabPaneTools.addAfterMenuItemFactory(this::newSliderboardTab),
                                 TabPaneTools.removeMenuItemFactory(),
                                 TabPaneTools.removeAllMenuItemFactory(false),
                                 exportTabMenuItemFactory(),
                                 exportAllTabMenuItemFactory(),
                                 importTabMenuItemFactory());

      sliderboardTabPane.getTabs().addListener((ListChangeListener<Tab>) change ->
      {
         while (change.next())
         {
            if (change.wasRemoved())
            {
               for (Tab removedTab : change.getRemoved())
               {
                  if (removedTab == initialTab)
                  {
                     JavaFXMissingTools.runLater(getClass(), () -> sliderboardTabPane.getTabs().add(initialTab));
                     initialTabHeader.setText("Default");
                     initialSliderboardPaneController.clear();
                  }
                  else
                  {
                     tabToControllerMap.remove(removedTab);
                  }
               }
            }
         }
      });

      sliderboardTabPane.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
      {
         if (oldValue != null)
            tabToControllerMap.get(oldValue).stop();
         if (newValue != null)
            tabToControllerMap.get(newValue).start();
      });

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
      window.setTitle("YoSliderboard controller");
      window.setScene(new Scene(sliderboardTabPane));
      window.initOwner(toolkit.getMainWindow());
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

   public void load(File file)
   {
      LogTools.info("Loading from file: " + file);

      try
      {
         YoSliderboardListDefinition definition = XMLTools.loadYoSliderboardListDefinition(new FileInputStream(file));
         setInput(definition);
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   public void save(File file)
   {
      LogTools.info("Saving to file: " + file);

      try
      {
         XMLTools.saveYoSliderboardListDefinition(new FileOutputStream(file), toYoSliderboardListDefinition());
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   private void setInput(YoSliderboardListDefinition input)
   {
      if (input.getYoSliderboards() == null || input.getYoSliderboards().isEmpty())
         return;

      ObservableList<Tab> tabs = sliderboardTabPane.getTabs();
      tabs.retainAll(initialTab);

      while (tabs.size() < input.getYoSliderboards().size())
         tabs.add(newSliderboardTab());

      List<YoSliderboardDefinition> sliderboards = input.getYoSliderboards();
      for (int i = 0; i < sliderboards.size(); i++)
      {
         tabToControllerMap.get(tabs.get(i)).setInput(sliderboards.get(i));
      }
   }

   public void close()
   {
      for (Tab tab : sliderboardTabPane.getTabs())
      {
         tabToControllerMap.get(tab).close();
      }

      sliderboardTabPane.getTabs().clear();
      tabToControllerMap.clear();
      tabToControllerMap.put(initialTab, initialSliderboardPaneController);
      window.close();
   }

   public Stage getWindow()
   {
      return window;
   }

   private Function<TabPane, MenuItem> exportTabMenuItemFactory()
   {
      return tabPane ->
      {
         Tab selectedTab = sliderboardTabPane.getSelectionModel().getSelectedItem();

         if (selectedTab == null)
            return null;

         FontAwesomeIconView exportIcon = new FontAwesomeIconView();
         exportIcon.getStyleClass().add("save-icon-view");
         MenuItem menuItem = new MenuItem("Export active tab...", exportIcon);

         menuItem.setOnAction(e ->
         {
            File result = SessionVisualizerIOTools.yoEntryConfigurationSaveFileDialog(owner);
            if (result != null)
               tabToControllerMap.get(selectedTab).save(result);
         });

         return menuItem;
      };
   }

   private Function<TabPane, MenuItem> exportAllTabMenuItemFactory()
   {
      return tabPane ->
      {
         Tab selectedTab = sliderboardTabPane.getSelectionModel().getSelectedItem();

         if (selectedTab == null)
            return null;

         FontAwesomeIconView exportIcon = new FontAwesomeIconView();
         exportIcon.getStyleClass().add("save-icon-view");
         MenuItem menuItem = new MenuItem("Export all tabs...", exportIcon);

         menuItem.setOnAction(e ->
         {
            File result = SessionVisualizerIOTools.yoEntryConfigurationSaveFileDialog(owner);
            if (result != null)
               exportAllTabs(result);
         });

         return menuItem;
      };
   }

   private Function<TabPane, MenuItem> importTabMenuItemFactory()
   {
      return tabPane ->
      {
         Tab selectedTab = sliderboardTabPane.getSelectionModel().getSelectedItem();

         if (selectedTab == null)
            return null;

         FontAwesomeIconView exportIcon = new FontAwesomeIconView();
         exportIcon.getStyleClass().add("load-icon-view");
         MenuItem menuItem = new MenuItem("Import tab(s)...", exportIcon);

         menuItem.setOnAction(e ->
         {
            File result = SessionVisualizerIOTools.yoEntryConfigurationOpenFileDialog(owner);
            if (result != null)
               importTabsAt(result, selectedTab);
         });

         return menuItem;
      };
   }

   private Tab newSliderboardTab()
   {
      try
      {
         Tab tab = new Tab("Sliderboard" + (sliderboardTabPane.getTabs().size() - 1));
         Label tabHeader = TabPaneTools.editableTabHeader(tab);
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_SLIDERBOARD_BCF2000_WINDOW_URL);
         Node node = loader.load();
         tab.setContent(node);
         YoBCF2000SliderboardWindowController controller = loader.getController();
         controller.nameProperty().bindBidirectional(tabHeader.textProperty());
         controller.initialize(window, toolkit);
         tabToControllerMap.put(tab, controller);
         return tab;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public void exportAllTabs(File file)
   {
      LogTools.info("Saving to file: " + file);

      try
      {
         XMLTools.saveYoSliderboardListDefinition(new FileOutputStream(file), toYoSliderboardListDefinition());
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   public void importTabsAt(File file, Tab insertionPoint)
   {
      LogTools.info("Loading from file: " + file);

      try
      {
         YoSliderboardListDefinition definition = XMLTools.loadYoSliderboardListDefinition(new FileInputStream(file));

         List<YoSliderboardDefinition> yoEntryLists = definition.getYoSliderboards();
         if (yoEntryLists == null || yoEntryLists.isEmpty())
            return;

         int startIndex = 0;

         if (isTabEmpty(insertionPoint))
         {
            tabToControllerMap.get(insertionPoint).setInput(yoEntryLists.get(0));
            startIndex++;
         }

         ObservableList<Tab> tabs = sliderboardTabPane.getTabs();
         int insertionIndex = tabs.indexOf(insertionPoint) + 1;

         for (int i = startIndex; i < yoEntryLists.size(); i++)
         {
            Tab newEmptyTab = newSliderboardTab();
            tabToControllerMap.get(newEmptyTab).setInput(yoEntryLists.get(i));
            tabs.add(insertionIndex, newEmptyTab);
            insertionIndex++;
         }
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   private boolean isTabEmpty(Tab query)
   {
      return tabToControllerMap.get(query).isEmpty();
   }

   private YoSliderboardListDefinition toYoSliderboardListDefinition()
   {
      YoSliderboardListDefinition definition = new YoSliderboardListDefinition();
      definition.setYoSliderboards(new ArrayList<>());
      for (Tab tab : sliderboardTabPane.getTabs())
      {
         definition.getYoSliderboards().add(tabToControllerMap.get(tab).toYoSliderboardDefinition());
      }
      return definition;
   }
}
