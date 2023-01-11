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

import com.google.common.base.Objects;

import de.jensd.fx.glyphs.fontawesome.FontAwesomeIconView;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.beans.value.ChangeListener;
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
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TabPaneTools;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController;

public class YoMultiBCF2000SliderboardWindowController
{
   /**
    * The name used for the default sliderboard. The default sliderboard is created when no
    * configuration is set or loaded from file. It is typically the first tab when multi sliderboards
    * have been created.
    */
   public static final String DEFAULT_SLIDERBOARD_NAME = "Default";
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

   private final List<Runnable> cleanupTasks = new ArrayList<>();

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

      ListChangeListener<Tab> preserveInitialTabListener = (ListChangeListener<Tab>) change ->
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
                     initialTabHeader.setText(DEFAULT_SLIDERBOARD_NAME);
                     initialSliderboardPaneController.clear();
                  }
                  else
                  {
                     tabToControllerMap.remove(removedTab);
                  }
               }
            }
         }
      };
      sliderboardTabPane.getTabs().addListener(preserveInitialTabListener);
      cleanupTasks.add(() -> sliderboardTabPane.getTabs().removeListener(preserveInitialTabListener));

      ChangeListener<? super Tab> controllerScheduler = (o, oldValue, newValue) ->
      {
         if (oldValue != null)
            tabToControllerMap.get(oldValue).stop();
         if (newValue != null)
            tabToControllerMap.get(newValue).start();
      };
      sliderboardTabPane.getSelectionModel().selectedItemProperty().addListener(controllerScheduler);
      cleanupTasks.add(() -> sliderboardTabPane.getSelectionModel().selectedItemProperty().removeListener(controllerScheduler));

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

      window.addEventHandler(WindowEvent.WINDOW_HIDING, e ->
      {
         LogTools.info("Stopping sliderboard binding.");
         tabToControllerMap.get(sliderboardTabPane.getSelectionModel().getSelectedItem()).stop();
      });
      window.addEventHandler(WindowEvent.WINDOW_SHOWING, e ->
      {
         LogTools.info("Starting sliderboard binding.");
         tabToControllerMap.get(sliderboardTabPane.getSelectionModel().getSelectedItem()).start();
      });

      window.setTitle("YoSliderboard controller");
      window.setScene(new Scene(sliderboardTabPane));
      window.initOwner(toolkit.getMainWindow());
   }

   public void setInput(YoSliderboardListDefinition input)
   {
      if (input.getYoSliderboards() == null || input.getYoSliderboards().isEmpty())
         return;

      // FIXME The initialTab is not properly handled 
      ObservableList<Tab> tabs = sliderboardTabPane.getTabs();
      tabs.retainAll(initialTab);

      while (tabs.size() < input.getYoSliderboards().size())
         tabs.add(newSliderboardTab());

      List<YoSliderboardDefinition> sliderboards = input.getYoSliderboards();
      for (int i = 0; i < sliderboards.size(); i++)
      {
         YoSliderboardDefinition sliderboard = sliderboards.get(i);
         if (Objects.equal(DEFAULT_SLIDERBOARD_NAME, sliderboard.getName()))
            initialSliderboardPaneController.setInput(sliderboard);
         else
            tabToControllerMap.get(tabs.get(i)).setInput(sliderboard);
      }
   }

   public void setSliderboard(YoSliderboardDefinition sliderboardDefinition)
   {
      Tab tab = findTabByName(sliderboardDefinition.getName());
      if (tab == null)
      {
         tab = newSliderboardTab();
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setInput(sliderboardDefinition);
   }

   public void closeSliderboard(String name)
   {
      Tab tabToRemove = findTabByName(name);
      if (tabToRemove != null)
         sliderboardTabPane.getTabs().remove(tabToRemove);
   }

   public void setButtonInput(String sliderboardName, YoButtonDefinition buttonDefinition)
   {
      Tab tab = findTabByName(sliderboardName);
      if (tab == null)
      {
         tab = newSliderboardTab();
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setButtonInput(buttonDefinition);
   }

   public void removeButtonInput(String sliderboardName, int buttonIndex)
   {
      Tab tab = findTabByName(sliderboardName);
      if (tab == null)
         return;
      tabToControllerMap.get(tab).removeButtonInput(buttonIndex);
   }

   public void setKnobInput(String sliderboardName, YoKnobDefinition knobDefinition)
   {
      Tab tab = findTabByName(sliderboardName);
      if (tab == null)
      {
         tab = newSliderboardTab();
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setKnobInput(knobDefinition);
   }

   public void removeKnobInput(String sliderboardName, int knobIndex)
   {
      Tab tab = findTabByName(sliderboardName);
      if (tab == null)
         return;
      tabToControllerMap.get(tab).removeKnobInput(knobIndex);
   }

   public void setSliderInput(String sliderboardName, YoSliderDefinition sliderDefinition)
   {
      Tab tab = findTabByName(sliderboardName);
      if (tab == null)
      {
         tab = newSliderboardTab();
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setSliderInput(sliderDefinition);
   }

   public void removeSliderInput(String sliderboardName, int sliderIndex)
   {
      Tab tab = findTabByName(sliderboardName);
      if (tab == null)
         return;
      tabToControllerMap.get(tab).removeSliderInput(sliderIndex);
   }

   private Tab findTabByName(String name)
   {
      if (name == null)
         return null;

      for (Tab tab : sliderboardTabPane.getTabs())
      {
         if (name.equals(tabToControllerMap.get(tab).nameProperty().get()))
         {
            return tab;
         }
      }
      return null;
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

   public void clear()
   {
      for (Tab tab : sliderboardTabPane.getTabs())
      {
         YoBCF2000SliderboardWindowController controller = tabToControllerMap.get(tab);
         controller.close();
      }

      sliderboardTabPane.getTabs().clear();
      tabToControllerMap.clear();
      tabToControllerMap.put(initialTab, initialSliderboardPaneController);
   }

   public void close()
   {
      cleanupTasks.forEach(Runnable::run);
      cleanupTasks.clear();
      clear();
      window.close();
      // FIXME We should keep track of the device we're using and avoid randomly all devices
      BCF2000SliderboardController.closeMidiDevices();
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
            File result = SessionVisualizerIOTools.yoSliderboardConfigurationSaveFileDialog(owner);
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
            File result = SessionVisualizerIOTools.yoSliderboardConfigurationSaveFileDialog(owner);
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
            File result = SessionVisualizerIOTools.yoSliderboardConfigurationOpenFileDialog(owner);
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
            sliderboardTabPane.getSelectionModel().select(insertionIndex);
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

   public YoSliderboardListDefinition toYoSliderboardListDefinition()
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
