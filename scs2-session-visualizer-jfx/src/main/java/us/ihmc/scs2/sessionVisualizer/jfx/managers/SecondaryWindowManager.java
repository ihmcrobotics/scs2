package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.ObservableList;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Rectangle2D;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import javafx.util.Pair;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.configuration.WindowConfigurationDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SCSGuiConfiguration;
import us.ihmc.scs2.sessionVisualizer.jfx.SecondaryWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.YoRegistryStatisticsPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000.YoBCF2000SliderboardWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.pattern.YoCompositePatternPropertyWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicPropertyWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SecondaryWindowManager implements Manager
{
   public static final String SECONDARY_CHART_WINDOW_TYPE = "SecondaryChartWindow";
   public static final String COMPOSITE_PATTERN_EDITOR_WINDOW_TYPE = "YoCompositeEditorWindow";
   public static final String GRAPHIC_EDITOR_WINDOW_TYPE = "YoGraphicEditorWindow";
   public static final String BCF2000_SLIDERBOARD_WINDOW_TYPE = "BC2000EditorWindow";
   public static final String REGISTRY_STATISTICS_WINDOW_TYPE = "YoRegistryStatisticsWindow";

   private final SessionVisualizerToolkit toolkit;

   private final Property<YoCompositePatternPropertyWindowController> yoCompositeEditor = new SimpleObjectProperty<>(this, "yoCompositeEditor", null);
   private final Property<YoGraphicPropertyWindowController> yoGraphicEditor = new SimpleObjectProperty<>(this, "yoGraphicEditor", null);
   private final Property<YoBCF2000SliderboardWindowController> bcf2000Sliderboard = new SimpleObjectProperty<>(this, "bcf2000Sliderboard", null);
   private final Property<YoRegistryStatisticsPaneController> yoRegistryStatistics = new SimpleObjectProperty<>(this, "yoRegistryStatistics", null);
   private final List<Stage> secondaryWindows = new ArrayList<>();
   private final List<SecondaryWindowController> secondaryWindowControllers = new ArrayList<>();
   private final JavaFXMessager messager;
   private final SessionVisualizerTopics topics;

   public SecondaryWindowManager(SessionVisualizerToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();

      messager.registerTopicListener(topics.getOpenWindowRequest(), this::openWindow);
   }

   @Override
   public boolean isSessionLoaded()
   {
      return false;
   }

   @Override
   public void startSession(Session session)
   {
   }

   public void loadSessionConfiguration(SCSGuiConfiguration configuration)
   {
      JavaFXMissingTools.runAndWait(getClass(), () ->
      {
         if (configuration.hasSecondaryWindowConfigurations())
         {
            List<WindowConfigurationDefinition> secondaryWindowConfigurations = configuration.getSecondaryWindowConfigurations();

            for (int i = 0; i < configuration.getNumberOfSecondaryYoChartGroupConfigurations(); i++)
            {
               WindowConfigurationDefinition secondaryWindowConfiguration = secondaryWindowConfigurations.get(i);
               Stage stage = newChartWindow(secondaryWindowConfiguration);
               messager.submitMessage(topics.getYoChartGroupLoadConfiguration(), new Pair<>(stage, configuration.getSecondaryYoChartGroupConfigurationFile(i)));
            }
         }
      });
   }

   public void saveSessionConfiguration(SCSGuiConfiguration configuration)
   {
      if (bcf2000Sliderboard.getValue() != null)
      {
         bcf2000Sliderboard.getValue().save(configuration.getYoSliderboardConfigurationFile());
      }

      for (SecondaryWindowController secondaryWindow : secondaryWindowControllers)
      {
         secondaryWindow.saveSessionConfiguration(configuration);
      }
   }

   @Override
   public void stopSession()
   {
      if (yoCompositeEditor.getValue() != null)
      {
         yoCompositeEditor.getValue().closeAndDispose();
         yoCompositeEditor.setValue(null);
      }

      if (yoGraphicEditor.getValue() != null)
      {
         yoGraphicEditor.getValue().closeAndDispose();
         yoGraphicEditor.setValue(null);
      }

      if (bcf2000Sliderboard.getValue() != null)
      {
         bcf2000Sliderboard.getValue().close();
         bcf2000Sliderboard.setValue(null);
      }

      if (yoRegistryStatistics.getValue() != null)
      {
         yoRegistryStatistics.getValue().close();
         yoRegistryStatistics.setValue(null);
      }

      secondaryWindows.forEach(secondaryWindow -> secondaryWindow.fireEvent(new WindowEvent(secondaryWindow, WindowEvent.WINDOW_CLOSE_REQUEST)));
      secondaryWindowControllers.forEach(SecondaryWindowController::closeAndDispose);
      secondaryWindowControllers.clear();
   }

   public void openWindow(Pair<String, Object> request)
   {
      if (request == null)
         return;

      String windowType = request.getKey();

      if (windowType == null || windowType.isEmpty())
         return;

      switch (windowType)
      {
         case COMPOSITE_PATTERN_EDITOR_WINDOW_TYPE:
            openYoCompositePatternEditor();
            break;
         case SECONDARY_CHART_WINDOW_TYPE:
            newChartWindow();
            break;
         case GRAPHIC_EDITOR_WINDOW_TYPE:
            openYoGraphicEditor();
            break;
         case BCF2000_SLIDERBOARD_WINDOW_TYPE:
            openBCF2000SliderboardWindow();
            return;
         case REGISTRY_STATISTICS_WINDOW_TYPE:
            openRegistryStatisticsWindow((String) request.getValue());
         default:
            LogTools.error("Unexpected value: " + windowType);
      }
   }

   public void openYoCompositePatternEditor()
   {
      if (yoCompositeEditor.getValue() != null)
      {
         yoCompositeEditor.getValue().showWindow();
         return;
      }

      try
      {
         FXMLLoader fxmlLoader = new FXMLLoader(SessionVisualizerIOTools.YO_COMPOSITE_PATTERN_PROPERTY_WINDOW_URL);
         fxmlLoader.load();
         YoCompositePatternPropertyWindowController controller = fxmlLoader.getController();
         controller.initialize(toolkit);
         yoCompositeEditor.setValue(controller);
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public Stage newChartWindow()
   {
      return newChartWindow(null);
   }

   public Stage newChartWindow(WindowConfigurationDefinition windowConfigurationDefinition)
   {
      Stage stage = newStage(windowConfigurationDefinition);

      try
      {
         // Loading template for secondary window
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.SECONDARY_WINDOW_URL);
         loader.load();
         SecondaryWindowController controller = loader.getController();
         controller.initialize(toolkit, stage);
         controller.setupChartGroup();
         secondaryWindowControllers.add(controller);
         stage.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
         {
            if (!e.isConsumed())
               secondaryWindowControllers.remove(controller);
         });
         controller.start();
         stage.show();
         return stage;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         stage.close();
         return null;
      }
   }

   private Stage newStage(WindowConfigurationDefinition definition)
   {
      Stage stage = new Stage();

      if (definition != null)
      {
         double positionX = definition.getPositionX();
         double positionY = definition.getPositionY();
         double width;
         if (definition.getWidth() > 0.0)
            width = definition.getWidth();
         else
            width = stage.getWidth();

         double height;
         if (definition.getHeight() > 0.0)
            height = definition.getHeight();
         else
            height = stage.getHeight();

         ObservableList<Screen> screensForRectangle = Screen.getScreensForRectangle(positionX, positionY, width, height);

         if (screensForRectangle.isEmpty())
         {
            // The window would be outside the visible bounds of the screens.
            // We'll reset the window so it appears in the middle of the primary screen.
            Screen primary = Screen.getPrimary();
            Rectangle2D visualBounds = primary.getVisualBounds();
            width = Math.min(width, visualBounds.getWidth());
            height = Math.min(height, visualBounds.getHeight());
            positionX = 0.5 * (visualBounds.getMinX() + visualBounds.getMaxX() - width);
            positionY = 0.5 * (visualBounds.getMinY() + visualBounds.getMaxY() - height);
         }

         stage.setX(positionX);
         stage.setY(positionY);

         if (definition.isMaximized())
         {
            stage.setMaximized(true);
         }
         else
         {
            stage.setWidth(width);
            stage.setHeight(height);
         }
      }

      secondaryWindows.add(stage);
      return stage;
   }

   public static WindowConfigurationDefinition toWindowConfigurationDefinition(Stage stage)
   {
      WindowConfigurationDefinition definition = new WindowConfigurationDefinition();
      if (stage.isMaximized())
      {
         definition.setMaximized(true);
         definition.setPositionX(stage.getX());
         definition.setPositionY(stage.getY());
         definition.setWidth(stage.getWidth());
         definition.setHeight(stage.getHeight());
      }
      else
      {
         definition.setMaximized(false);
         definition.setPositionX(stage.getX());
         definition.setPositionY(stage.getY());
         definition.setWidth(stage.getWidth());
         definition.setHeight(stage.getHeight());
      }
      return definition;
   }

   private void openYoGraphicEditor()
   {
      if (yoGraphicEditor.getValue() != null)
      {
         yoGraphicEditor.getValue().showWindow();
         return;
      }

      try
      {
         FXMLLoader fxmlLoader = new FXMLLoader(SessionVisualizerIOTools.YO_GRAPHIC_PROPERTY_URL);
         fxmlLoader.load();
         YoGraphicPropertyWindowController controller = fxmlLoader.getController();
         controller.initialize(toolkit);
         yoGraphicEditor.setValue(controller);
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void openBCF2000SliderboardWindow()
   {
      if (bcf2000Sliderboard.getValue() != null)
      {
         bcf2000Sliderboard.getValue().showWindow();
         return;
      }

      try
      {
         FXMLLoader fxmlLoader = new FXMLLoader(SessionVisualizerIOTools.YO_SLIDERBOARD_BCF2000_WINDOW_URL);
         fxmlLoader.load();
         YoBCF2000SliderboardWindowController controller = fxmlLoader.getController();
         controller.initialize(toolkit);
         bcf2000Sliderboard.setValue(controller);
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void openRegistryStatisticsWindow(String registryFullname)
   {
      YoRegistry registry = toolkit.getYoManager().getRootRegistry().findRegistry(new YoNamespace(registryFullname));

      if (registry == null)
      {
         LogTools.error("Could not find a registry from the name: {}", registryFullname);
         return;
      }

      if (yoRegistryStatistics.getValue() != null)
      {
         yoRegistryStatistics.getValue().setInput(registry);
         yoRegistryStatistics.getValue().showWindow();
      }
      else
      {
         try
         {
            FXMLLoader fxmlLoader = new FXMLLoader(SessionVisualizerIOTools.YO_REGISTRY_STATISTICS_URL);
            fxmlLoader.load();
            YoRegistryStatisticsPaneController controller = fxmlLoader.getController();
            controller.initialize(toolkit);
            controller.setInput(registry);
            yoRegistryStatistics.setValue(controller);
            controller.showWindow();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }
}
