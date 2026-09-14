package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.stage.Stage;
import javafx.stage.Window;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.sessionVisualizer.jfx.MainWindowController;
import us.ihmc.scs2.sessionVisualizer.jfx.Scene3DBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest;
import us.ihmc.scs2.sessionVisualizer.jfx.session.OpenSessionControlsRequest.SessionType;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/**
 * Session-controls auto-show/bind at {@link MultiSessionManager}: after a Log session or MCAP log
 * session is started without the user opening the pane (the CLI {@code -l} path), the matching
 * window is showing and bound.
 */
@Execution(ExecutionMode.SAME_THREAD)
public class MultiSessionManagerTest
{
   private static volatile boolean fxReady = false;
   private static SessionVisualizerToolkit toolkit;
   private static MultiSessionManager multiSessionManager;
   private static Stage mainWindow;

   @BeforeAll
   static void startHarness() throws Exception
   {
      try
      {
         System.setProperty("yo.allowRepeatingSubname", "true");
         System.setProperty("glass.platform", "Monocle");
         System.setProperty("monocle.platform", "Headless");
         System.setProperty("prism.order", "sw");
         System.setProperty("java.awt.headless", "true");
         System.setProperty("__GL_SYNC_TO_VBLANK", "0");

         CountDownLatch latch = new CountDownLatch(1);
         Platform.startup(latch::countDown);
         if (!latch.await(15, TimeUnit.SECONDS))
            throw new IllegalStateException("Timed out waiting for the JavaFX toolkit to start.");
         fxReady = true;
      }
      catch (IllegalStateException alreadyStarted)
      {
         fxReady = true;
      }
      catch (Throwable t)
      {
         fxReady = false;
         System.err.println("[MultiSessionManagerTest] JavaFX toolkit unavailable; skipping: " + t);
      }

      assumeTrue(fxReady, "JavaFX toolkit unavailable");

      AtomicReference<Exception> failure = new AtomicReference<>();
      JavaFXMissingTools.runAndWait(MultiSessionManagerTest.class, () ->
      {
         try
         {
            mainWindow = new Stage();
            Scene3DBuilder scene3DBuilder = new Scene3DBuilder();
            scene3DBuilder.addDefaultLighting();
            toolkit = new SessionVisualizerToolkit(mainWindow, scene3DBuilder.getRoot());

            FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.MAIN_WINDOW_URL);
            Parent mainPane = loader.load();
            MainWindowController mainWindowController = loader.getController();
            mainWindowController.initialize(new SessionVisualizerWindowToolkit(mainWindow, toolkit));
            mainWindow.setScene(new Scene(mainPane));
            mainWindow.show();

            multiSessionManager = new MultiSessionManager(toolkit, mainWindowController);
         }
         catch (Exception e)
         {
            failure.set(e);
         }
      });
      if (failure.get() != null)
         throw failure.get();
   }

   @BeforeEach
   void assumeHarness()
   {
      assumeTrue(fxReady && multiSessionManager != null, "JavaFX harness unavailable");
   }

   @AfterEach
   void stopSession()
   {
      if (multiSessionManager == null)
         return;
      JavaFXMissingTools.runAndWait(getClass(), () ->
      {
         multiSessionManager.stopSession(false, true);
         closeSessionControlsWindows();
      });
   }

   @AfterAll
   static void tearDownHarness()
   {
      if (multiSessionManager == null)
         return;
      JavaFXMissingTools.runAndWait(MultiSessionManagerTest.class, () ->
      {
         multiSessionManager.stopSession(false, true);
         multiSessionManager.shutdown();
         closeSessionControlsWindows();
         if (mainWindow != null)
            mainWindow.close();
      });
      multiSessionManager = null;
      toolkit = null;
      mainWindow = null;
   }

   @Test
   public void testStartLogSessionShowsBoundLogSessionControls() throws Exception
   {
      File logDirectory = SessionControlsTestLog.createLogDirectory("cli-bind-log");
      LogSession logSession = new LogSession(logDirectory, null);

      startSessionOnFx(logSession);

      Stage controls = findOpenStage("Log session controls");
      assertNotNull(controls, "Log session controls should be showing after a CLI-style start");
      assertTrue(controls.isShowing());
      assertBound(controls, SessionControlsTestLog.SESSION_NAME, logDirectory);
   }

   @Test
   public void testStartSameTypeAgainKeepsOneRefreshedWindow() throws Exception
   {
      File firstDirectory = SessionControlsTestLog.createLogDirectory("cli-bind-log-a", "first-log");
      File secondDirectory = SessionControlsTestLog.createLogDirectory("cli-bind-log-b", "second-log");

      startSessionOnFx(new LogSession(firstDirectory, null));
      Stage first = findOpenStage("Log session controls");
      assertNotNull(first);
      assertBound(first, "first-log", firstDirectory);

      JavaFXMissingTools.runAndWait(getClass(), () -> multiSessionManager.stopSession(false, true));
      startSessionOnFx(new LogSession(secondDirectory, null));

      Stage second = findOpenStage("Log session controls");
      assertNotNull(second);
      assertTrue(second.isShowing());
      assertEquals(first, second, "Same-type start should refresh the existing window");
      assertBound(second, "second-log", secondDirectory);
   }

   @Test
   public void testEndSessionClearsPaneAndLeavesWindowOpen() throws Exception
   {
      File logDirectory = SessionControlsTestLog.createLogDirectory("cli-bind-end");
      startSessionOnFx(new LogSession(logDirectory, null));

      Stage controls = findOpenStage("Log session controls");
      assertNotNull(controls);
      Button endSession = findButton(controls.getScene().getRoot(), "End session");
      JavaFXMissingTools.runAndWait(getClass(), endSession::fire);

      assertTrue(controls.isShowing(), "End session should leave the window open");
      assertEquals("N/D", labelBeside(controls.getScene().getRoot(), "Session name:").getText());
      assertEquals("N/D", labelBeside(controls.getScene().getRoot(), "Date:").getText());
      assertEquals("N/D", labelBeside(controls.getScene().getRoot(), "Log path:").getText());
      assertTrue(endSession.isDisabled(), "End session should disable itself after clearing the pane");
   }

   @Test
   public void testOpenLogMenuAfterCliStartShowsAlreadyBoundWindow() throws Exception
   {
      File logDirectory = SessionControlsTestLog.createLogDirectory("cli-bind-menu");
      startSessionOnFx(new LogSession(logDirectory, null));

      Stage controls = findOpenStage("Log session controls");
      assertNotNull(controls);

      JavaFXMissingTools.runAndWait(getClass(),
                                    () -> toolkit.getMessager()
                                                 .submitMessage(toolkit.getTopics().getOpenSessionControlsRequest(),
                                                                new OpenSessionControlsRequest(mainWindow, SessionType.LOG)));

      Stage afterMenu = findOpenStage("Log session controls");
      assertEquals(controls, afterMenu);
      assertTrue(afterMenu.isShowing());
      assertBound(afterMenu, SessionControlsTestLog.SESSION_NAME, logDirectory);
   }

   private void startSessionOnFx(Session session)
   {
      JavaFXMissingTools.runAndWait(getClass(), () -> multiSessionManager.startSession(session, null));
   }

   private static void closeSessionControlsWindows()
   {
      for (Window window : List.copyOf(Window.getWindows()))
      {
         if (window instanceof Stage stage)
         {
            String title = stage.getTitle();
            if ("Log session controls".equals(title) || "MCAP Log session controls".equals(title))
               stage.close();
         }
      }
   }

   private static Stage findOpenStage(String title)
   {
      for (Window window : Window.getWindows())
      {
         if (window instanceof Stage stage && title.equals(stage.getTitle()) && stage.isShowing())
            return stage;
      }
      return null;
   }

   private static void assertBound(Stage controls, String sessionName, File logPath)
   {
      assertEquals(sessionName, labelBeside(controls.getScene().getRoot(), "Session name:").getText());
      assertNotEquals("N/D", labelBeside(controls.getScene().getRoot(), "Date:").getText());
      assertEquals(logPath.getAbsolutePath(), labelBeside(controls.getScene().getRoot(), "Log path:").getText());

      Button endSession = findButton(controls.getScene().getRoot(), "End session");
      assertNotNull(endSession);
      assertFalse(endSession.isDisabled(), "End session should be enabled once the window is bound");

      Node slider = findSlider(controls.getScene().getRoot());
      assertNotNull(slider);
      assertFalse(slider.isDisabled(), "Scrub slider should be enabled once the window is bound");
   }

   private static Label labelBeside(Parent root, String caption)
   {
      List<Label> labels = new ArrayList<>();
      collect(root, Label.class, labels);
      for (int i = 0; i < labels.size() - 1; i++)
      {
         if (caption.equals(labels.get(i).getText()))
            return labels.get(i + 1);
      }
      throw new AssertionError("No label beside '" + caption + "'");
   }

   private static Button findButton(Parent root, String text)
   {
      List<Button> buttons = new ArrayList<>();
      collect(root, Button.class, buttons);
      return buttons.stream().filter(b -> text.equals(b.getText())).findFirst().orElse(null);
   }

   private static Node findSlider(Parent root)
   {
      List<Node> nodes = new ArrayList<>();
      collect(root, Node.class, nodes);
      return nodes.stream().filter(n -> n.getClass().getSimpleName().contains("Slider")).findFirst().orElse(null);
   }

   private static <T> void collect(Node node, Class<T> type, List<T> out)
   {
      if (type.isInstance(node))
         out.add(type.cast(node));
      if (node instanceof Parent parent)
      {
         for (Node child : parent.getChildrenUnmodifiable())
            collect(child, type, out);
      }
   }
}
