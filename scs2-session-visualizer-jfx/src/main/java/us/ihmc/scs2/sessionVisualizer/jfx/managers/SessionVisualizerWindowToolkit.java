package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.stage.Stage;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.BufferedJavaFXMessager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

public class SessionVisualizerWindowToolkit
{
   private final Stage window;
   private final SessionVisualizerToolkit globalToolkit;
   private final ChartZoomManager chartZoomManager;
   private final WindowShortcutManager windowShortcutManager;

   public SessionVisualizerWindowToolkit(Stage window, SessionVisualizerToolkit globalToolkit)
   {
      this.window = window;
      this.globalToolkit = globalToolkit;
      chartZoomManager = new ChartZoomManager(window, getMessager(), getTopics());
      windowShortcutManager = new WindowShortcutManager(window, getMessager(), getTopics());
   }

   public void start()
   {
      chartZoomManager.start();
      windowShortcutManager.start();
   }

   public void stop()
   {
      chartZoomManager.stop();
      windowShortcutManager.stop();
   }

   public Stage getWindow()
   {
      return window;
   }

   public SessionVisualizerToolkit getGlobalToolkit()
   {
      return globalToolkit;
   }

   public BufferedJavaFXMessager getMessager()
   {
      return globalToolkit.getMessager();
   }

   public SessionVisualizerTopics getTopics()
   {
      return globalToolkit.getTopics();
   }

   public YoCompositeSearchManager getYoCompositeSearchManager()
   {
      return globalToolkit.getYoCompositeSearchManager();
   }

   public BackgroundExecutorManager getBackgroundExecutorManager()
   {
      return globalToolkit.getBackgroundExecutorManager();
   }

   public ChartDataManager getChartDataManager()
   {
      return globalToolkit.getChartDataManager();
   }

   public ChartRenderManager getChartRenderManager()
   {
      return globalToolkit.getChartRenderManager();
   }

   public ChartZoomManager getChartZoomManager()
   {
      return chartZoomManager;
   }

   public YoManager getYoManager()
   {
      return globalToolkit.getYoManager();
   }

   public KeyFrameManager getKeyFrameManager()
   {
      return globalToolkit.getKeyFrameManager();
   }

   public YoGroupFX getYoGraphicFXRootGroup()
   {
      return globalToolkit.getYoGraphicFXRootGroup();
   }

   public ReferenceFrameManager getReferenceFrameManager()
   {
      return globalToolkit.getReferenceFrameManager();
   }

   public YoGraphicFXManager getYoGraphicFXManager()
   {
      return globalToolkit.getYoGraphicFXManager();
   }
}
