package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import java.lang.reflect.Field;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.Consumer;

import javafx.event.Event;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.Menu;
import javafx.scene.control.MenuItem;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.testfx.api.FxToolkit;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager;

/**
 * Regression test for a bug where {@link RunMenuController}'s playback-rate/run-max-duration text
 * fields could never actually be edited: the fields submitted to the {@link Session} on every
 * keystroke, and the session's own periodic {@code SessionProperties} broadcast (see
 * {@code Session#reportActiveMode()}) unconditionally overwrote the field's displayed value via
 * {@code Platform.runLater(...)}, regardless of whether the field currently had focus. In practice
 * this meant the field snapped back to its old value before the user could finish typing.
 * <p>
 * The fix: the fields now only submit to the session on Enter, and losing focus without pressing
 * Enter discards the edit (reverts the field to its pre-edit value) instead of submitting or
 * keeping the uncommitted text.
 * <p>
 * These fields live inside a {@link CustomMenuItem}, wired up via
 * {@code MenuTools.configureTextFieldForCustomMenuItem()}, which installs its own key filter that
 * consumes ENTER and shifts focus to close out its "edit mode" - so a real Enter key press never
 * reaches the field's action-event machinery the way a plain {@code TextField} would. The fixture
 * below reproduces that wrapper (content + a focusable container) and drives Enter via a real
 * {@link KeyEvent}, not a synthetic {@code ActionEvent}, so this test actually exercises that path.
 * <p>
 * Note: every JavaFX object in this test is created after {@link FxToolkit#registerPrimaryStage()}
 * (which boots the JavaFX Toolkit) - constructing e.g. a {@code TextField} any earlier throws
 * {@code IllegalStateException: Toolkit not initialized}.
 */
public class RunMenuControllerTest
{
   private TestSession session;

   @AfterEach
   public void tearDown()
   {
      if (session != null && session.hasSessionStarted())
         session.shutdownSession();
   }

   @Tag("javafx-headless")
   @Test
   public void testFieldSurvivesLiveSessionUpdatesWhileFocused() throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();

      session = new TestSession();
      // Ticks every 10ms in PAUSE mode; the SessionProperties broadcast is throttled to ~500ms
      // (see Session#sessionPropertiesPublishPeriod), so several broadcasts happen over the ~1s
      // window this test waits below.
      assertTrue(session.startSessionThread());

      Fixture fixture = FxToolkit.setupFixture(() -> buildFixture(session));

      Callable<Void> beginEdit = () ->
      {
         fixture.playbackRealTimeRateTextField().requestFocus();
         assertTrue(fixture.playbackRealTimeRateTextField().isFocused(), "Test setup: the field should be focusable in a shown stage");

         // Simulate the user typing a new value without having committed it yet (no Enter, no focus
         // change). This is the in-progress-edit state that the live session broadcast used to clobber.
         simulateTyping(fixture.playbackRealTimeRateTextField(), "42.0");
         return null;
      };
      FxToolkit.setupFixture(beginEdit);

      // Give the session's real periodic broadcast (throttled to ~500ms) a chance to fire more than
      // once while the field is still focused and mid-edit, off the FX thread so Platform.runLater
      // tasks queued from the session's background thread actually get to run.
      Thread.sleep(900);

      Callable<Void> assertNotClobberedWhileFocused = () ->
      {
         assertEquals("42.0",
                      fixture.playbackRealTimeRateTextField().getText(),
                      "The field's in-progress edit should survive live session-properties broadcasts while it has focus");
         assertEquals(1.0,
                      session.getSessionProperties().getPlaybackRealTimeRate(),
                      "Nothing should have been submitted to the session yet - only Enter commits");
         return null;
      };
      FxToolkit.setupFixture(assertNotClobberedWhileFocused);
   }

   @Tag("javafx-headless")
   @Test
   public void testLosingFocusWithoutEnterDiscardsTheEditInsteadOfCommittingIt() throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();

      session = new TestSession();

      Fixture fixture = FxToolkit.setupFixture(() -> buildFixture(session));

      Callable<Void> editThenClickAway = () ->
      {
         fixture.playbackRealTimeRateTextField().requestFocus();
         // Typed but never confirmed with Enter.
         simulateTyping(fixture.playbackRealTimeRateTextField(), "42.0");
         fixture.focusSink().requestFocus();
         return null;
      };
      FxToolkit.setupFixture(editThenClickAway);

      Callable<Void> assertDiscarded = () ->
      {
         assertEquals(1.0, session.getSessionProperties().getPlaybackRealTimeRate(), "Nothing should be submitted to the session without pressing Enter");
         assertEquals("1.0",
                      fixture.playbackRealTimeRateTextField().getText(),
                      "Losing focus without pressing Enter should revert the field to its pre-edit value, not keep the uncommitted text");
         return null;
      };
      FxToolkit.setupFixture(assertDiscarded);
   }

   /**
    * Regression test: a real Enter key press must commit the typed value, even though
    * {@code MenuTools.configureTextFieldForCustomMenuItem()} consumes the ENTER {@link KeyEvent} and
    * shifts focus away from the field before the field's own action-event machinery would normally
    * fire. An earlier version of the fix relied solely on {@code TextField#setOnAction} plus a
    * focus-lost listener that reverted uncommitted edits - which meant a real Enter press looked
    * exactly like clicking away without committing, so the typed value was discarded instead of
    * applied.
    */
   @Tag("javafx-headless")
   @Test
   public void testEnterKeyPressCommitsTheTypedValueThroughTheMenuItemWrapper() throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();

      session = new TestSession();

      Fixture fixture = FxToolkit.setupFixture(() -> buildFixture(session));

      Callable<Void> typeAndPressEnter = () ->
      {
         fixture.playbackRealTimeRateTextField().requestFocus();
         simulateTyping(fixture.playbackRealTimeRateTextField(), "3.5");
         Event.fireEvent(fixture.playbackRealTimeRateTextField(), new KeyEvent(KeyEvent.KEY_PRESSED, "\r", "\r", KeyCode.ENTER, false, false, false, false));
         return null;
      };
      FxToolkit.setupFixture(typeAndPressEnter);

      Callable<Void> assertCommitted = () ->
      {
         assertEquals(3.5, session.getSessionProperties().getPlaybackRealTimeRate(), "A real Enter key press should commit the typed value to the session");
         assertEquals("3.5", fixture.playbackRealTimeRateTextField().getText(), "The field should keep showing the committed value, not revert it");
         return null;
      };
      FxToolkit.setupFixture(assertCommitted);
   }

   /**
    * Simulates a user typing into a formatted field the way real keystrokes do: via
    * {@code TextInputControl#replaceText(int, int, String)}, not {@code setText(String)}.
    * <p>
    * This distinction matters: {@code TextInputControl} only converts text into the
    * {@code TextFormatter}'s value on commit (its own internal focus-lost listener, or an explicit
    * {@code commitValue()} call) - real typing goes through {@code replaceText}/{@code insertText},
    * which leaves {@code TextFormatter#getValue()} untouched until then. {@code setText(String)}
    * instead goes through the control's {@code text} property setter, which - unlike real typing -
    * syncs the formatter's value immediately. A test that types via {@code setText(...)} would give
    * a false pass for exactly the bug this fixture exists to catch (RunMenuController reading a
    * stale {@code formatter.getValue()} before it has been synced to the just-typed text).
    */
   private static void simulateTyping(TextField field, String text)
   {
      field.replaceText(0, field.getLength(), text);
   }

   /**
    * Builds a real {@link RunMenuController} wired to real JavaFX controls and a real (minimal)
    * {@link Session}, with only the {@link SessionVisualizerWindowToolkit}/{@link SessionVisualizerToolkit}
    * boundary mocked out - that boundary is heavyweight application wiring (3D scene, robot models,
    * background executors, ...) unrelated to the bug under test. Must run on the FX Application Thread.
    * <p>
    * The text fields are wrapped in a {@link CustomMenuItem} with a focusable container, mirroring
    * production's FXML layout, so {@code MenuTools.configureTextFieldForCustomMenuItem()} (called by
    * {@code initialize()}) behaves exactly as it does in the real menu instead of NPE-ing on a
    * missing {@code CustomMenuItem#getContent()} or no-op-ing a focus shift with nowhere to go.
    */
   private static Fixture buildFixture(Session session) throws Exception
   {
      RunMenuController controller = new RunMenuController();

      TextField playbackRealTimeRateTextField = new TextField();
      TextField runMaxDurationTextField = new TextField();
      Button focusSink = new Button("elsewhere");

      CustomMenuItem playbackRealTimeRateMenuItem = new CustomMenuItem();
      HBox playbackRealTimeRateContent = new HBox(playbackRealTimeRateTextField);
      playbackRealTimeRateMenuItem.setContent(playbackRealTimeRateContent);

      CustomMenuItem runMaxDurationMenuItem = new CustomMenuItem();
      HBox runMaxDurationContent = new HBox(runMaxDurationTextField);
      runMaxDurationMenuItem.setContent(runMaxDurationContent);

      setField(controller, "menu", new Menu());
      setField(controller, "playbackRealTimeRateMenuItem", playbackRealTimeRateMenuItem);
      setField(controller, "runMaxDurationMenuItem", runMaxDurationMenuItem);
      setField(controller, "simulateAtRealTimeCheckMenuItem", new CheckMenuItem());
      setField(controller, "resetMenuItem", new MenuItem());
      setField(controller, "playbackRealTimeRateTextField", playbackRealTimeRateTextField);
      setField(controller, "runMaxDurationTextField", runMaxDurationTextField);

      SCS2Messager messager = new SCS2Messager();
      messager.startMessager();
      SessionVisualizerTopics topics = new SessionVisualizerTopics();
      topics.setupTopics();

      SessionVisualizerToolkit globalToolkit = mock(SessionVisualizerToolkit.class);
      SessionVisualizerWindowToolkit toolkit = mock(SessionVisualizerWindowToolkit.class);
      when(toolkit.getMessager()).thenReturn(messager);
      when(toolkit.getTopics()).thenReturn(topics);
      when(toolkit.getGlobalToolkit()).thenReturn(globalToolkit);
      doAnswer(invocation ->
      {
         SessionChangeListener listener = invocation.getArgument(0);
         listener.sessionChanged(null, session);
         return null;
      }).when(toolkit).addAndTriggerSessionChangedListener(any());

      // Stands in for the real Menu popup's internal item container: MenuTools' Enter handling calls
      // requestFocus() on CustomMenuItem#getContent()#getParent(), so that parent must actually be
      // able to take focus for the test to faithfully reproduce the real focus shift.
      VBox menuItemContainer = new VBox(focusSink, playbackRealTimeRateContent, runMaxDurationContent);
      menuItemContainer.setFocusTraversable(true);
      Stage stage = new Stage();
      stage.setScene(new Scene(menuItemContainer, 200, 200));
      stage.show();

      controller.initialize(toolkit);

      return new Fixture(playbackRealTimeRateTextField, runMaxDurationTextField, focusSink);
   }

   private record Fixture(TextField playbackRealTimeRateTextField, TextField runMaxDurationTextField, Button focusSink)
   {
   }

   private static void setField(Object target, String fieldName, Object value) throws Exception
   {
      Field field = RunMenuController.class.getDeclaredField(fieldName);
      field.setAccessible(true);
      field.set(target, value);
   }

   private static class TestSession extends Session
   {
      @Override
      protected double doSpecificRunTick()
      {
         return 0.0;
      }

      @Override
      public void addGraphicsAddedCallback(Consumer<List<YoGraphicDefinition>> addedGraphicsConsumer)
      {
      }

      @Override
      public String getSessionName()
      {
         return "RunMenuControllerTestSession";
      }

      @Override
      public List<RobotDefinition> getRobotDefinitions()
      {
         return Collections.emptyList();
      }

      @Override
      public List<TerrainObjectDefinition> getTerrainObjectDefinitions()
      {
         return Collections.emptyList();
      }
   }
}
