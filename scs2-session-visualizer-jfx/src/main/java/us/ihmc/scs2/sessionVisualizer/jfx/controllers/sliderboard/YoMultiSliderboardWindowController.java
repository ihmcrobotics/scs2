package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

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
import us.ihmc.scs2.definition.yoSlider.YoSliderboardType;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000.YoBCF2000SliderboardWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.xtouchcompact.YoXTouchCompactSliderboardWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TabPaneTools;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;
import us.ihmc.scs2.sessionVisualizer.sliderboard.BCF2000SliderboardController;
import us.ihmc.scs2.sessionVisualizer.sliderboard.XTouchCompactSliderboardController;

public class YoMultiSliderboardWindowController
{
   /**
    * The name used for the default sliderboard. The default sliderboard is created when no
    * configuration is set or loaded from file. It is typically the first tab when multi sliderboards
    * have been created.
    */
   public static final String DEFAULT_SLIDERBOARD_NAME = "Default";
   public static final YoSliderboardType DEFAULT_SLIDERBOARD_TYPE = YoSliderboardType.BCF2000;
   @FXML
   private TabPane sliderboardTabPane;

   private final Map<Tab, YoSliderboardWindowControllerInterface> tabToControllerMap = new HashMap<>();

   private SessionVisualizerToolkit toolkit;

   private Stage window;
   private Window owner;
   
   private YoSliderboardType initialType = YoSliderboardType.BCF2000;

   private final List<Runnable> cleanupTasks = new ArrayList<>();

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      owner = toolkit.getMainWindow();
      window = new Stage(StageStyle.UTILITY);
      

      MenuTools.setupContextMenu(sliderboardTabPane,
                                 TabPaneTools.addBeforeMenuItemFactory(this::newBFC2000SliderboardTab, "Add BFC2000 tab before"),
                                 TabPaneTools.addAfterMenuItemFactory(this::newBFC2000SliderboardTab,  "Add BFC2000 tab after"),
                                 TabPaneTools.addBeforeMenuItemFactory(this::newXtouchCompactTab, "Add XTouch tab before"),
                                 TabPaneTools.addAfterMenuItemFactory(this::newXtouchCompactTab, "Add Xtouch tab after"),
                                 TabPaneTools.removeMenuItemFactory(),
                                 TabPaneTools.removeAllMenuItemFactory(false),
                                 exportTabMenuItemFactory(),
                                 exportAllTabMenuItemFactory(),
                                 importTabMenuItemFactory());

      ListChangeListener<Tab> preserveOneTabListener = (ListChangeListener<Tab>) change ->
      {
         while (change.next())
         {
            if (change.wasRemoved())
            {
               for (Tab removedTab : change.getRemoved())
               {
                  var removed = tabToControllerMap.remove(removedTab);
                  
                  if(removed != null)
                  {
                     initialType = removed.getType();
                  }
                  
                  if(tabToControllerMap.isEmpty())
                  {
                     newInitialTab();
                  }
               }
            }
         }
      };
      sliderboardTabPane.getTabs().addListener(preserveOneTabListener);
      cleanupTasks.add(() -> sliderboardTabPane.getTabs().removeListener(preserveOneTabListener));

      ChangeListener<? super Tab> controllerScheduler = (o, oldValue, newValue) ->
      {
         if (oldValue != null)
            tabToControllerMap.get(oldValue).stop();
         if (newValue != null)
            tabToControllerMap.get(newValue).start();
         
         window.sizeToScene();
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
      tabs.clear();


      List<YoSliderboardDefinition> sliderboards = input.getYoSliderboards();
      for (int i = 0; i < sliderboards.size(); i++)
      {
         YoSliderboardDefinition sliderboard = sliderboards.get(i);
         setSliderboard(sliderboard);
      }
   }

   public void setSliderboard(YoSliderboardDefinition sliderboardDefinition)
   {
      Tab tab = findTabByNameAndType(sliderboardDefinition.getName(), sliderboardDefinition.getType());
      if (tab == null)
      {
         tab = newSliderboardTab(sliderboardDefinition.getType());         
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setInput(sliderboardDefinition);
   }

   public void closeSliderboard(String name, YoSliderboardType type)
   {
      Tab tabToRemove = findTabByNameAndType(name, type);
      if (tabToRemove != null)
         sliderboardTabPane.getTabs().remove(tabToRemove);
   }

   public void setButtonInput(String sliderboardName, YoSliderboardType sliderboardType, YoButtonDefinition buttonDefinition)
   {
      Tab tab = findTabByNameAndType(sliderboardName, sliderboardType);
      if (tab == null)
      {
         tab = newSliderboardTab(sliderboardType);
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setButtonInput(buttonDefinition);
   }

   public void removeButtonInput(String sliderboardName, YoSliderboardType sliderboardType,  int buttonIndex)
   {
      Tab tab = findTabByNameAndType(sliderboardName, sliderboardType);
      if (tab == null)
         return;
      tabToControllerMap.get(tab).removeButtonInput(buttonIndex);
   }

   public void setKnobInput(String sliderboardName, YoSliderboardType sliderboardType, YoKnobDefinition knobDefinition)
   {
      Tab tab = findTabByNameAndType(sliderboardName, sliderboardType);
      if (tab == null)
      {
         tab = newSliderboardTab(sliderboardType);
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setKnobInput(knobDefinition);
   }

   public void removeKnobInput(String sliderboardName, YoSliderboardType sliderboardType, int knobIndex)
   {
      Tab tab = findTabByNameAndType(sliderboardName, sliderboardType);
      if (tab == null)
         return;
      tabToControllerMap.get(tab).removeKnobInput(knobIndex);
   }

   public void setSliderInput(String sliderboardName, YoSliderboardType sliderboardType, YoSliderDefinition sliderDefinition)
   {
      Tab tab = findTabByNameAndType(sliderboardName, sliderboardType);
      if (tab == null)
      {
         tab = newSliderboardTab(sliderboardType);
         sliderboardTabPane.getTabs().add(tab);
      }
      tabToControllerMap.get(tab).setSliderInput(sliderDefinition);
   }

   public void removeSliderInput(String sliderboardName, YoSliderboardType sliderboardType, int sliderIndex)
   {
      Tab tab = findTabByNameAndType(sliderboardName, sliderboardType);
      if (tab == null)
         return;
      tabToControllerMap.get(tab).removeSliderInput(sliderIndex);
   }

   private Tab findTabByNameAndType(String name, YoSliderboardType type)
   {
      if (name == null)
         return null;

      for (Tab tab : sliderboardTabPane.getTabs())
      {
         YoSliderboardWindowControllerInterface controller = tabToControllerMap.get(tab);
         if (name.equals(controller.nameProperty().get()) && controller.getType() == type)
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
         YoSliderboardWindowControllerInterface controller = tabToControllerMap.get(tab);
         controller.close();
      }

      sliderboardTabPane.getTabs().clear();
      tabToControllerMap.clear();
      
      newInitialTab();
   }

   private void newInitialTab()
   {
      var tab = newSliderboardTab(initialType);
      
      sliderboardTabPane.getTabs().add(tab);
      sliderboardTabPane.getSelectionModel().select(0);
   }

   public void close()
   {
      cleanupTasks.forEach(Runnable::run);
      cleanupTasks.clear();
      clear();
      window.close();
      // FIXME We should keep track of the device we're using and avoid randomly all devices
      BCF2000SliderboardController.closeMidiDevices();
      XTouchCompactSliderboardController.closeMidiDevices();
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

   private Tab newSliderboardTab(YoSliderboardType type)
   {
      switch(type)
      {
         case BCF2000:
            return newBFC2000SliderboardTab();
         case XTOUCHCOMPACT:
            return newXtouchCompactTab();
         default:
            throw new RuntimeException("Invalid sliderboard type: " + type);
      }
      
   }
   private Tab newBFC2000SliderboardTab()
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
   
   private Tab newXtouchCompactTab()
   {
      try
      {
         Tab tab = new Tab("XTouch Compact" + (sliderboardTabPane.getTabs().size() - 1));
         Label tabHeader = TabPaneTools.editableTabHeader(tab);
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.YO_SLIDERBOARD_XTOUCHCOMPACT_WINDOW_URL);
         Node node = loader.load();
         tab.setContent(node);
         YoSliderboardWindowControllerInterface controller = loader.getController();
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
            Tab newEmptyTab = newSliderboardTab(yoEntryLists.get(i).getType());
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
   
   public void ensureTab(YoSliderboardType type)
   {
      for(var controller : tabToControllerMap.entrySet())
      {
         if(controller.getValue().getType() == type)
         {
            return;
         }
      }
      
      var tab = newSliderboardTab(type);
      
      int index = sliderboardTabPane.getTabs().size();
      sliderboardTabPane.getTabs().add(tab);
      sliderboardTabPane.getSelectionModel().select(index);
   }

}
