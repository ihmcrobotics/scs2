package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search;

import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.FXCollections;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.ListView;
import javafx.scene.control.Spinner;
import org.junit.jupiter.api.Test;
import org.testfx.api.FxToolkit;
import org.testfx.util.WaitForAsyncUtils;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A cell must keep following live values when the list is refreshed, must relink when a new session replaces the
 * registry (every session creates new YoVariable instances that carry the same names as the previous session's), and
 * its control must actually be laid out by the list, otherwise the values update but nothing is visible.
 */
public class YoCompositeListCellLinkingTest
{
   private static class Sess
   {
      final YoDouble source;
      final YoSharedBuffer buffer;
      final LinkedYoRegistry linked;
      final YoDouble uiVar;

      Sess(double initial)
      {
         YoRegistry sessionRoot = new YoRegistry("root");
         source = new YoDouble("x", sessionRoot);
         source.set(initial);
         buffer = new YoSharedBuffer(sessionRoot, 16);
         buffer.writeBuffer();
         YoRegistry uiRoot = new YoRegistry("root"); // what YoManager.startSession creates every session
         linked = buffer.newLinkedYoRegistry(uiRoot);
         uiVar = (YoDouble) uiRoot.findVariable("root.x");
      }

      void publish(double value)
      {
         source.set(value);
         buffer.writeBuffer();
         buffer.prepareLinkedBuffersForPull();
         linked.pull(); // what YoManager.handleImpl does on every animation frame
         WaitForAsyncUtils.waitForFxEvents();
      }
   }

   private static YoComposite wrap(Sess s)
   {
      return new YoComposite(YoCompositeSearchManager.yoVariablePattern, s.uiVar);
   }

   private static Double shown(YoCompositeListCell cell)
   {
      return ((Spinner<Double>) cell.getGraphic()).getValue();
   }

   private YoCompositeListCell newCell(LinkedYoRegistry[] current) throws Exception
   {
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();
      YoManager yoManager = new YoManager()
      {
         @Override
         public LinkedYoRegistry getLinkedRootRegistry()
         {
            return current[0];
         }
      };
      return FxToolkit.setupFixture(() -> new YoCompositeListCell(yoManager,
                                                                  new SimpleObjectProperty<>(YoNameDisplay.SHORT_NAME),
                                                                  new SimpleObjectProperty<>(6),
                                                                  new ListView<>()));
   }

   @Test
   public void sameSessionNewWrapperKeepsUpdating() throws Exception
   {
      Sess a = new Sess(1.0);
      LinkedYoRegistry[] current = {a.linked};
      YoCompositeListCell cell = newCell(current);

      FxToolkit.setupFixture(() -> cell.updateItem(wrap(a), false));
      assertEquals(1.0, shown(cell), 1e-12);

      FxToolkit.setupFixture(() -> cell.updateItem(wrap(a), false)); // list refresh, brand new wrapper, same variable
      a.publish(2.0);
      assertEquals(2.0, shown(cell), 1e-12, "same session, refreshed wrapper");
   }

   @Test
   public void newSessionSameNamesKeepsUpdating() throws Exception
   {
      Sess a = new Sess(1.0);
      LinkedYoRegistry[] current = {a.linked};
      YoCompositeListCell cell = newCell(current);

      FxToolkit.setupFixture(() -> cell.updateItem(wrap(a), false));
      a.publish(2.0);
      assertEquals(2.0, shown(cell), 1e-12, "session A sanity check");

      // New session: new registry, new YoVariable instances, same names.
      Sess b = new Sess(10.0);
      current[0] = b.linked;
      FxToolkit.setupFixture(() -> cell.updateItem(wrap(b), false));
      b.publish(20.0);
      assertEquals(20.0, shown(cell), 1e-12, "session B: cell should follow the NEW session's variable");
   }

   @Test
   @SuppressWarnings("unchecked")
   public void controlInsideListViewIsLaidOutAndShowsLiveValues() throws Exception
   {
      Sess a = new Sess(1.0);
      LinkedYoRegistry[] current = {a.linked};
      if (!FxToolkit.isFXApplicationThreadRunning())
         FxToolkit.registerPrimaryStage();
      YoManager yoManager = new YoManager()
      {
         @Override
         public LinkedYoRegistry getLinkedRootRegistry()
         {
            return current[0];
         }
      };

      ListView<YoComposite> listView = new ListView<>();
      listView.setCellFactory(param -> new YoCompositeListCell(yoManager, new SimpleObjectProperty<>(YoNameDisplay.SHORT_NAME), new SimpleObjectProperty<>(6), param));
      listView.setItems(FXCollections.observableArrayList(wrap(a)));
      FxToolkit.setupFixture(() ->
                             {
                                new Scene(listView, 400, 300);
                                listView.applyCss();
                                listView.layout();
                             });
      WaitForAsyncUtils.waitForFxEvents();

      for (double value : new double[] {2.0, 3.0})
      {
         a.publish(value);
         FxToolkit.setupFixture(() ->
                                {
                                   listView.applyCss();
                                   listView.layout();
                                   Set<Node> cells = listView.lookupAll(".yo-variable-list-cell");
                                   Spinner<Double> spinner = null;
                                   for (Node cell : cells)
                                   {
                                      if (cell instanceof YoCompositeListCell c && c.getGraphic() != null)
                                         spinner = (Spinner<Double>) c.getGraphic();
                                   }
                                   assertNotNull(spinner, "the list should have created a cell with a control");
                                   // An unmanaged graphic is ignored by the cell's layout: it stays 0x0, so nothing is drawn even though the value updates.
                                   assertTrue(spinner.isManaged(), "the control must be managed by the cell's layout");
                                   assertTrue(spinner.getWidth() > 0.0 && spinner.getHeight() > 0.0, "the control must have a size, was " + spinner.getWidth() + "x" + spinner.getHeight());
                                   assertEquals(value, Double.parseDouble(spinner.getEditor().getText()), 1e-12, "the displayed text must follow the live value");
                                });
      }
   }
}
