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
import javafx.stage.Window;
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
   private static final double SECONDARY_WINDOW_POSITION_OFFSET = 30.0;

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
               Stage stage = newChartWindow(null, secondaryWindowConfiguration);
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

   public void openWindow(NewWindowRequest request)
   {
      if (request == null)
         return;

      switch (request.windowType)
      {
         case NewWindowRequest.COMPOSITE_PATTERN_EDITOR_WINDOW_TYPE:
            openYoCompositePatternEditor(request.requestSource);
            break;
         case NewWindowRequest.SECONDARY_CHART_WINDOW_TYPE:
            newChartWindow(request.requestSource);
            break;
         case NewWindowRequest.GRAPHIC_EDITOR_WINDOW_TYPE:
            openYoGraphicEditor(request.requestSource);
            break;
         case NewWindowRequest.BCF2000_SLIDERBOARD_WINDOW_TYPE:
            openBCF2000SliderboardWindow(request.requestSource);
            return;
         case NewWindowRequest.REGISTRY_STATISTICS_WINDOW_TYPE:
            openRegistryStatisticsWindow(request.requestSource, (String) request.additionalData);
         default:
            LogTools.error("Unexpected value: " + request.windowType);
      }
   }

   public void openYoCompositePatternEditor(Window requestSource)
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
         initializeSecondaryWindowWithOwner(requestSource, controller.getWindow());
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public Stage newChartWindow(Window requestSource)
   {
      return newChartWindow(requestSource, null);
   }

   public Stage newChartWindow(Window requestSource, WindowConfigurationDefinition windowConfigurationDefinition)
   {
      Stage stage = newStage(requestSource, windowConfigurationDefinition);

      try
      {
         // Loading template for secondary window
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.SECONDARY_WINDOW_URL);
         loader.load();
         SecondaryWindowController controller = loader.getController();
         controller.initialize(new SessionVisualizerWindowToolkit(stage, toolkit));
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

   private Stage newStage(Window requestSource, WindowConfigurationDefinition definition)
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
      else
      {
         initializeSecondaryWindowWithOwner(requestSource, stage);
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

   private void openYoGraphicEditor(Window requestSource)
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
         initializeSecondaryWindowWithOwner(requestSource, controller.getWindow());
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void openBCF2000SliderboardWindow(Window requestSource)
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
         initializeSecondaryWindowWithOwner(requestSource, controller.getWindow());
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void openRegistryStatisticsWindow(Window requestSource, String registryFullname)
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
            initializeSecondaryWindowWithOwner(requestSource, controller.getWindow());
            controller.showWindow();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   public static void initializeSecondaryWindowWithOwner(Window owner, Stage secondary)
   {
      if (owner == null)
         return;
      secondary.setX(owner.getX() + SECONDARY_WINDOW_POSITION_OFFSET);
      secondary.setY(owner.getY() + 30.0);
   }

   public static class NewWindowRequest
   {
      public static final String REGISTRY_STATISTICS_WINDOW_TYPE = "YoRegistryStatisticsWindow";
      public static final String BCF2000_SLIDERBOARD_WINDOW_TYPE = "BC2000EditorWindow";
      public static final String GRAPHIC_EDITOR_WINDOW_TYPE = "YoGraphicEditorWindow";
      public static final String COMPOSITE_PATTERN_EDITOR_WINDOW_TYPE = "YoCompositeEditorWindow";
      public static final String SECONDARY_CHART_WINDOW_TYPE = "SecondaryChartWindow";

      private final String windowType;
      private final Window requestSource;
      private final Object additionalData;

      public NewWindowRequest(String windowType, Window requestSource)
      {
         this(windowType, requestSource, null);
      }

      public NewWindowRequest(String windowType, Window requestSource, Object additionalData)
      {
         this.windowType = windowType;
         this.requestSource = requestSource;
         this.additionalData = additionalData;
      }

      public static NewWindowRequest registryStatisticWindow(Window requestSource, YoRegistry registry)
      {
         return new NewWindowRequest(REGISTRY_STATISTICS_WINDOW_TYPE, requestSource, registry.getNamespace().toString());
      }

      public static NewWindowRequest bcf2000SliderboardWindow(Window requestSource)
      {
         return new NewWindowRequest(BCF2000_SLIDERBOARD_WINDOW_TYPE, requestSource);
      }

      public static NewWindowRequest graphicEditorWindow(Window requestSource)
      {
         return new NewWindowRequest(GRAPHIC_EDITOR_WINDOW_TYPE, requestSource);
      }

      public static NewWindowRequest compositePatternEditorWindow(Window requestSource)
      {
         return new NewWindowRequest(COMPOSITE_PATTERN_EDITOR_WINDOW_TYPE, requestSource);
      }

      public static NewWindowRequest chartWindow(Window requestSource)
      {
         return new NewWindowRequest(SECONDARY_CHART_WINDOW_TYPE, requestSource);
      }
   }
}
