package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.stage.Stage;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.SCS2JavaFXMessager;
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
      chartZoomManager = new ChartZoomManager(window, this, getMessager(), getTopics());
      windowShortcutManager = new WindowShortcutManager(window, this, getMessager(), getTopics());
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

   public SCS2JavaFXMessager getMessager()
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

   public SessionDataPreferenceManager getSessionDataPreferenceManager()
   {
      return globalToolkit.getSessionDataPreferenceManager();
   }

   public Session getSession()
   {
      return globalToolkit.getSession();
   }

   public void addSessionChangedListener(SessionChangeListener listener)
   {
      globalToolkit.addSessionChangedListener(listener);
   }

   /**
    * Registers the listener and immediately invokes it with {@code (null, getSession())}, so callers
    * don't need to separately handle "attach to whatever session is already active".
    *
    * @param listener the listener to add.
    */
   public void addAndTriggerSessionChangedListener(SessionChangeListener listener)
   {
      addSessionChangedListener(listener);
      listener.sessionChanged(null, getSession());
   }

   public boolean removeSessionChangedListener(SessionChangeListener listener)
   {
      return globalToolkit.removeSessionChangedListener(listener);
   }
}
