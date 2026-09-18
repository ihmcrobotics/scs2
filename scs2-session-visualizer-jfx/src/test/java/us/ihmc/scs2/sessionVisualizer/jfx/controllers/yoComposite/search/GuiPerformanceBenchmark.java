package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import com.jfoenix.controls.JFXTrimSlider;
import javafx.application.Platform;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.FXCollections;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.ListView;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;
import org.testfx.api.FxToolkit;
import org.testfx.util.WaitForAsyncUtils;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CoalescingFXTaskScheduler;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;

/**
 * Opt-in timing harness for the GUI work that shows up in the profiler. It is skipped unless the environment variable
 * {@code SCS2_BENCHMARKS=true} is set, so it never slows down the regular test run:
 *
 * <pre>
 * SCS2_BENCHMARKS=true ./gradlew :scs2:scs2-session-visualizer-jfx:scs2-session-visualizer-jfx-test:test \
 *     --tests '*GuiPerformanceBenchmark' -i | grep BENCH
 * </pre>
 * <p>
 * It measures CPU-side work only (property updates, CSS and layout passes), not rendering, so it is meant to compare
 * two versions of the code on the same machine, not to predict an absolute frame rate. Run it before and after a
 * change, on an otherwise idle machine, and compare the medians.
 * </p>
 */
@EnabledIfEnvironmentVariable(named = "SCS2_BENCHMARKS", matches = "true")
public class GuiPerformanceBenchmark
{
   private static final int NUMBER_OF_VARIABLES = 300;
   private static final int WARMUP_FRAMES = 300;
   private static final int MEASURED_FRAMES = 600;

   // ---------------------------------------------------------------------------------------------------------------
   // Fixtures
   // ---------------------------------------------------------------------------------------------------------------

   private static class Session
   {
      final YoDouble[] sources = new YoDouble[NUMBER_OF_VARIABLES];
      final YoSharedBuffer buffer;
      final LinkedYoRegistry linked;
      final List<YoComposite> composites = new ArrayList<>();

      Session()
      {
         YoRegistry sessionRoot = new YoRegistry("root");
         for (int i = 0; i < sources.length; i++)
            sources[i] = new YoDouble("variable" + i, sessionRoot);
         buffer = new YoSharedBuffer(sessionRoot, 16);
         buffer.writeBuffer();
         YoRegistry uiRoot = new YoRegistry("root"); // what YoManager.startSession creates for every session
         linked = buffer.newLinkedYoRegistry(uiRoot);
         for (int i = 0; i < sources.length; i++)
            composites.add(newComposite(i, uiRoot));
      }

      private static YoComposite newComposite(int i, YoRegistry uiRoot)
      {
         return new YoComposite(YoCompositeSearchManager.yoVariablePattern, (YoDouble) uiRoot.findVariable("root.variable" + i));
      }

      /** Wraps the same YoVariables in brand new YoComposite instances, like every YoCompositeSearchManager refresh. */
      List<YoComposite> rewrap()
      {
         List<YoComposite> result = new ArrayList<>();
         for (YoComposite composite : composites)
            result.add(new YoComposite(composite.getPattern(), (YoDouble) composite.getYoComponents().get(0)));
         return result;
      }

      /** What a session tick followed by YoManager.handleImpl() does: new values reach the UI variables. */
      void publishFrame(int frame)
      {
         for (int i = 0; i < sources.length; i++)
            sources[i].set(frame * 0.001 + i);
         buffer.writeBuffer();
         buffer.prepareLinkedBuffersForPull();
         linked.pull();
      }
   }

   private static class Gui
   {
      final Session session;
      final LinkedYoRegistry[] currentLinkedRegistry;
      final ListView<YoComposite> listView = new ListView<>();
      final Scene scene;

      Gui(Session session) throws Exception
      {
         this.session = session;
         currentLinkedRegistry = new LinkedYoRegistry[] {session.linked};
         YoManager yoManager = new YoManager()
         {
            @Override
            public LinkedYoRegistry getLinkedRootRegistry()
            {
               return currentLinkedRegistry[0];
            }
         };
         listView.setCellFactory(param -> new YoCompositeListCell(yoManager,
                                                                  new SimpleObjectProperty<>(YoNameDisplay.SHORT_NAME),
                                                                  new SimpleObjectProperty<>(6),
                                                                  param));
         listView.setItems(FXCollections.observableArrayList(session.composites));
         scene = onFX(() ->
                      {
                         Scene s = new Scene(listView, 400, 800);
                         layoutPass();
                         return s;
                      });
      }

      /** Approximates the CSS + layout phases of a JavaFX pulse for this scene. */
      void layoutPass()
      {
         listView.applyCss();
         listView.layout();
      }
   }

   private static void ensureToolkit() throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();
   }

   private static <T> T onFX(Supplier<T> supplier) throws Exception
   {
      return FxToolkit.setupFixture(supplier::get);
   }

   /** Runs the body on the FX thread and blocks until it is done, so per-iteration timing has no thread hops in it. */
   private static long[] timeOnFX(int warmup, int measured, Frame frame) throws Exception
   {
      long[] samples = new long[measured];
      CountDownLatch done = new CountDownLatch(1);
      Throwable[] failure = new Throwable[1];
      Platform.runLater(() ->
                        {
                           try
                           {
                              for (int i = 0; i < warmup; i++)
                                 frame.run(i);
                              for (int i = 0; i < measured; i++)
                              {
                                 long start = System.nanoTime();
                                 frame.run(warmup + i);
                                 samples[i] = System.nanoTime() - start;
                              }
                           }
                           catch (Throwable t)
                           {
                              failure[0] = t;
                           }
                           finally
                           {
                              done.countDown();
                           }
                        });
      done.await();
      if (failure[0] != null)
         throw new RuntimeException(failure[0]);
      return samples;
   }

   private interface Frame
   {
      void run(int frame);
   }

   private static void report(String scenario, long[] nanos)
   {
      long[] sorted = nanos.clone();
      Arrays.sort(sorted);
      double mean = Arrays.stream(sorted).average().orElse(Double.NaN);
      System.out.printf("BENCH %-62s median=%8.1f us   p95=%8.1f us   mean=%8.1f us   (n=%d)%n",
                        scenario,
                        sorted[sorted.length / 2] / 1000.0,
                        sorted[(int) (sorted.length * 0.95)] / 1000.0,
                        mean / 1000.0,
                        sorted.length);
   }

   private static int countCells(Gui gui)
   {
      int count = 0;
      for (Node node : gui.listView.lookupAll(".yo-variable-list-cell"))
      {
         if (node instanceof YoCompositeListCell cell && cell.getGraphic() != null)
            count++;
      }
      return count;
   }

   // ---------------------------------------------------------------------------------------------------------------
   // 1. Cost of one frame of the variables panel while a log plays back
   // ---------------------------------------------------------------------------------------------------------------

   @Test
   public void variablesPanelFrameCost() throws Exception
   {
      ensureToolkit();
      Gui gui = new Gui(new Session());
      System.out.println("BENCH variables panel: " + NUMBER_OF_VARIABLES + " variables, " + onFX(() -> countCells(gui)) + " cells with a live control");

      // Baseline: nothing changes, so there is nothing to do. Anything above this is the price of live values.
      report("panel, values NOT changing (baseline)", timeOnFX(WARMUP_FRAMES, MEASURED_FRAMES, f -> gui.layoutPass()));

      // The realistic case: every frame the session publishes new values, then the pulse runs CSS + layout.
      report("panel, values changing every frame [CURRENT CODE]", timeOnFX(WARMUP_FRAMES, MEASURED_FRAMES, f ->
      {
         gui.session.publishFrame(f);
         gui.layoutPass();
      }));

      // Emulates the old setManaged(false) change. NOT a valid alternative: the controls then have size 0x0 and are
      // not drawn at all, so the time saved is simply the time of not showing the values.
      onFX(() ->
           {
              for (Node node : gui.listView.lookupAll(".yo-variable-list-cell"))
              {
                 if (node instanceof YoCompositeListCell cell && cell.getGraphic() != null)
                    cell.getGraphic().setManaged(false);
              }
              return null;
           });
      report("panel, values changing, controls unmanaged [INVALID: not drawn]", timeOnFX(WARMUP_FRAMES, MEASURED_FRAMES, f ->
      {
         gui.session.publishFrame(f);
         gui.layoutPass();
      }));
   }

   // ---------------------------------------------------------------------------------------------------------------
   // 2. Cost of a list refresh (search box typed in, registry changed, ...) that produces new YoComposite wrappers
   // ---------------------------------------------------------------------------------------------------------------

   @Test
   public void listRefreshCost() throws Exception
   {
      ensureToolkit();
      Session session = new Session();
      Gui gui = new Gui(session);
      int warmup = 50;
      int measured = 300;

      // Building the wrappers is not part of what is being measured, so they are prepared up front.
      List<List<YoComposite>> rewrapped = new ArrayList<>();
      for (int i = 0; i < warmup + measured; i++)
         rewrapped.add(session.rewrap());

      // Does the "same item" check in YoCompositeListCell.updateItem() ever keep a control across a refresh? ListView
      // resets every cell to empty before giving it the new item, which clears the cell's remembered item, so this
      // currently prints false: a refresh rebuilds the controls even when the YoVariables are unchanged.
      Node controlBefore = onFX(() -> firstControl(gui));
      onFX(() ->
           {
              gui.listView.setItems(FXCollections.observableArrayList(session.rewrap()));
              gui.layoutPass();
              return null;
           });
      Node controlAfter = onFX(() -> firstControl(gui));
      System.out.println("BENCH info: control kept across setItems() with same variables = " + (controlBefore != null && controlBefore == controlAfter));

      report("setItems(), new wrappers around the same variables", timeOnFX(warmup, measured, f ->
      {
         gui.listView.setItems(FXCollections.observableArrayList(rewrapped.get(f)));
         gui.layoutPass();
      }));

      // Different YoVariables every time, which is what a new session does.
      Session other = new Session();
      boolean[] useOther = {true};
      report("setItems(), different variables (new session)", timeOnFX(warmup, measured, f ->
      {
         Session next = useOther[0] ? other : session;
         gui.currentLinkedRegistry[0] = next.linked;
         useOther[0] = !useOther[0];
         gui.listView.setItems(FXCollections.observableArrayList(next.composites));
         gui.layoutPass();
      }));
   }

   private static Node firstControl(Gui gui)
   {
      for (Node node : gui.listView.lookupAll(".yo-variable-list-cell"))
      {
         if (node instanceof YoCompositeListCell cell && cell.getGraphic() != null)
            return cell.getGraphic();
      }
      return null;
   }

   // ---------------------------------------------------------------------------------------------------------------
   // 3. FX thread load from the log position slider while a log plays back (session publishes at ~100Hz)
   // ---------------------------------------------------------------------------------------------------------------

   @Test
   public void logPositionSliderThrottleCost() throws Exception
   {
      ensureToolkit();
      JFXTrimSlider slider = new JFXTrimSlider();
      slider.setMin(0);
      slider.setMax(1_000_000);
      onFX(() ->
           {
              Scene scene = new Scene(new javafx.scene.layout.AnchorPane(slider), 600, 100);
              slider.applyCss();
              slider.layout();
              return scene;
           });

      int publishRateHz = 100;
      int seconds = 4;

      // Before: every publish queues its own runLater that moves the slider.
      AtomicInteger naiveTasks = new AtomicInteger();
      AtomicLong naiveNanos = new AtomicLong();
      AtomicInteger position = new AtomicInteger();
      runPublisher(publishRateHz, seconds, () ->
      {
         int target = position.incrementAndGet();
         Platform.runLater(() ->
                           {
                              long start = System.nanoTime();
                              slider.setValue(target);
                              slider.applyCss();
                              slider.layout();
                              naiveNanos.addAndGet(System.nanoTime() - start);
                              naiveTasks.incrementAndGet();
                           });
      });

      // After: publishes only request an update, coalesced and capped at 30Hz, reading the live position when it runs.
      AtomicInteger scheduledTasks = new AtomicInteger();
      AtomicLong scheduledNanos = new AtomicLong();
      position.set(0);
      CoalescingFXTaskScheduler scheduler = new CoalescingFXTaskScheduler(() ->
                                                                          {
                                                                             long start = System.nanoTime();
                                                                             slider.setValue(position.get());
                                                                             slider.applyCss();
                                                                             slider.layout();
                                                                             scheduledNanos.addAndGet(System.nanoTime() - start);
                                                                             scheduledTasks.incrementAndGet();
                                                                          },
                                                                          Platform::runLater,
                                                                          TimeUnit.MILLISECONDS.toNanos(33));
      runPublisher(publishRateHz, seconds, () ->
      {
         position.incrementAndGet();
         scheduler.request();
      });

      System.out.printf("BENCH slider @%dHz publish, %ds: per-publish runLater      -> %4d FX tasks, %7.1f ms of FX thread time%n",
                        publishRateHz,
                        seconds,
                        naiveTasks.get(),
                        naiveNanos.get() / 1.0e6);
      System.out.printf("BENCH slider @%dHz publish, %ds: CoalescingFXTaskScheduler -> %4d FX tasks, %7.1f ms of FX thread time%n",
                        publishRateHz,
                        seconds,
                        scheduledTasks.get(),
                        scheduledNanos.get() / 1.0e6);
   }

   private static void runPublisher(int rateHz, int seconds, Runnable publish) throws Exception
   {
      long periodNanos = TimeUnit.SECONDS.toNanos(1) / rateHz;
      long end = System.nanoTime() + TimeUnit.SECONDS.toNanos(seconds);
      long next = System.nanoTime();
      while (System.nanoTime() < end)
      {
         publish.run();
         next += periodNanos;
         long sleep = next - System.nanoTime();
         if (sleep > 0)
            TimeUnit.NANOSECONDS.sleep(sleep);
      }
      WaitForAsyncUtils.waitForFxEvents();
   }
}
