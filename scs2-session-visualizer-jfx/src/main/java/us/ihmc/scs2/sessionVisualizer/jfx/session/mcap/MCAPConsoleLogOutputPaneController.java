package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import javafx.beans.binding.DoubleBinding;
import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;
import javafx.scene.control.SelectionMode;
import javafx.scene.input.Clipboard;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.KeyCode;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.text.Font;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.scs2.session.mcap.MCAPConsoleLogManager.MCAPConsoleLogItem;
import us.ihmc.scs2.session.mcap.MCAPConsoleLogManager.MCAPLogLevel;
import us.ihmc.scs2.session.mcap.MCAPLogFileReader;
import us.ihmc.scs2.session.mcap.MCAPLogSession;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.simulation.SpyList;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoLong;

import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.Collections;
import java.util.Comparator;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Consumer;

public class MCAPConsoleLogOutputPaneController
{
   private static final int MAX_UPDATES_PER_FRAME = 10;

   @FXML
   private Pane mainPane;
   @FXML
   private ListView<MCAPConsoleLogItem> consoleOutputListView;

   private EventHandler<MouseEvent> goToLogEventMouseEventHandler;
   private Consumer<YoBufferPropertiesReadOnly> scrollLastLogItemListener;
   private MCAPLogSession session;

   private final ConcurrentLinkedQueue<MCAPConsoleLogItemListCell> listCellsToUpdate = new ConcurrentLinkedQueue<>();

   private final ObservedAnimationTimer observedAnimationTimer = new ObservedAnimationTimer()
   {

      @Override
      public void handleImpl(long now)
      {
         MCAPConsoleLogItemListCell cell;
         for (int i = 0; i < MAX_UPDATES_PER_FRAME; i++)
         {
            if ((cell = listCellsToUpdate.poll()) == null)
               break;
            cell.updateTextFill();
         }
      }
   };

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      MenuTools.setupContextMenu(consoleOutputListView, mcapConsoleLogItemListView ->
      {
         MenuItem goToLogItem = new MenuItem("Go to Log Item (Double Click)");
         goToLogItem.setOnAction(event ->
                                 {
                                    MCAPConsoleLogItem selectedLogItem = mcapConsoleLogItemListView.getSelectionModel().getSelectedItem();
                                    if (selectedLogItem != null)
                                       gotToLogItem(session, selectedLogItem);
                                 });
         return goToLogItem;
      }, mcapConsoleLogItemListView ->
                                 {
                                    MenuItem copyLogItem = new MenuItem("Copy Log Item (Ctrl+C)");
                                    copyLogItem.setOnAction(event -> copySelectedLogItemsToClipboard(mcapConsoleLogItemListView));
                                    return copyLogItem;
                                 });

      consoleOutputListView.setOnKeyPressed(event ->
                                            {
                                               if (event.getCode().equals(KeyCode.C) && event.isControlDown())
                                               {
                                                  copySelectedLogItemsToClipboard(consoleOutputListView);
                                                  event.consume();
                                               }
                                            });
   }

   public void startSession(MCAPLogSession session)
   {
      this.session = session;
      // Setup the console output
      MCAPLogFileReader mcapLogFileReader = session.getMCAPLogFileReader();
      consoleOutputListView.setCellFactory(param -> new MCAPConsoleLogItemListCell(param, session, mcapLogFileReader.getCurrentTimestamp()));
      consoleOutputListView.getItems().clear();
      SpyList<MCAPConsoleLogItem> sessionLogItems = mcapLogFileReader.getConsoleLogManager().getAllConsoleLogItems();
      consoleOutputListView.getItems().setAll(sessionLogItems);
      sessionLogItems.addListener((change) ->
                                  {
                                     if (change.wasAdded())
                                     {
                                        // Doing that outside the JavaFX thread to ensure we have the right size.
                                        int newItemsSize = sessionLogItems.size();
                                        JavaFXMissingTools.runLater(getClass(), () ->
                                        {
                                           // Log items only get added, so we can just add the new items.
                                           // This manner is robust to concurrent modifications and missing an update.
                                           for (int i = consoleOutputListView.getItems().size(); i < newItemsSize; i++)
                                           {
                                              consoleOutputListView.getItems().add(sessionLogItems.get(i));
                                           }
                                        });
                                     }
                                  });

      consoleOutputListView.getSelectionModel().setSelectionMode(SelectionMode.MULTIPLE);

      MutableLong requestingLogTimestamp = new MutableLong(Long.MIN_VALUE);

      goToLogEventMouseEventHandler = event ->
      {
         if (event.getButton() != MouseButton.PRIMARY)
            return;
         if (event.getClickCount() != 2)
            return;
         if (!event.isStillSincePress())
            return;

         MCAPConsoleLogItem selectedLogItem = consoleOutputListView.getSelectionModel().getSelectedItem();
         if (selectedLogItem == null)
            return;
         requestingLogTimestamp.setValue(selectedLogItem.logTime());

         gotToLogItem(session, selectedLogItem);
      };
      scrollLastLogItemListener = new Consumer<>()
      {
         private long lastTimestamp = Long.MIN_VALUE;
         private int lastSearchResult = Integer.MIN_VALUE;

         @Override
         public void accept(YoBufferPropertiesReadOnly bufferProperties)
         {
            long currentTimestamp = session.getMCAPLogFileReader().getCurrentTimestamp().getValue();

            if (requestingLogTimestamp.getValue() != Long.MIN_VALUE && requestingLogTimestamp.getValue() == currentTimestamp)
            {
               requestingLogTimestamp.setValue(Long.MIN_VALUE);
               lastTimestamp = currentTimestamp;
               return;
            }

            if (currentTimestamp == lastTimestamp)
               return;
            lastTimestamp = currentTimestamp;

            int searchResult = Collections.binarySearch(consoleOutputListView.getItems(),
                                                        new MCAPConsoleLogItem(currentTimestamp, null, null, null, null, null, 0),
                                                        Comparator.comparingLong(MCAPConsoleLogItem::logTime));
            if (searchResult < 0)
               searchResult = -searchResult - 1;
            if (searchResult == lastSearchResult)
               return;
            lastSearchResult = searchResult;
            int finalSearchResult = Math.max(0, searchResult - 2);
            JavaFXMissingTools.runLater(MCAPConsoleLogOutputPaneController.this.getClass(), () -> consoleOutputListView.scrollTo(finalSearchResult));
         }
      };

      consoleOutputListView.addEventFilter(MouseEvent.MOUSE_CLICKED, goToLogEventMouseEventHandler);
      session.addCurrentBufferPropertiesListener(scrollLastLogItemListener);
      observedAnimationTimer.start();
   }

   private static void gotToLogItem(MCAPLogSession session, MCAPConsoleLogItem selectedLogItem)
   {
      int logIndex = session.getMCAPLogFileReader().getIndexFromTimestamp(selectedLogItem.logTime());
      session.submitLogPositionRequest(logIndex);
   }

   private static void copySelectedLogItemsToClipboard(ListView<MCAPConsoleLogItem> mcapConsoleLogItemListView)
   {
      ObservableList<MCAPConsoleLogItem> selectedItems = mcapConsoleLogItemListView.getSelectionModel().getSelectedItems();
      StringBuilder logItemString = new StringBuilder();
      for (MCAPConsoleLogItem logItem : selectedItems)
      {
         logItemString.append("[%s] [%s] [%s]: %s\n".formatted(logItem.logLevel(), logItem.instant(), logItem.processName(), logItem.message()));
      }

      ClipboardContent clipboardContent = new ClipboardContent();
      clipboardContent.putString(logItemString.toString());
      Clipboard.getSystemClipboard().setContent(clipboardContent);
   }

   public void stopSession()
   {
      consoleOutputListView.getItems().clear();
      if (goToLogEventMouseEventHandler != null)
         consoleOutputListView.removeEventFilter(MouseEvent.MOUSE_CLICKED, goToLogEventMouseEventHandler);
      goToLogEventMouseEventHandler = null;
      if (scrollLastLogItemListener != null)
         session.removeCurrentBufferPropertiesListener(scrollLastLogItemListener);
      scrollLastLogItemListener = null;
      observedAnimationTimer.stop();
      listCellsToUpdate.clear();
   }

   private class MCAPConsoleLogItemListCell extends javafx.scene.control.ListCell<MCAPConsoleLogItem>
   {
      private final Color defaultColor = Color.BLACK;
      private final Map<MCAPLogLevel, Color> logLevelToColorMap = Map.of(MCAPLogLevel.UNKNOWN,
                                                                         defaultColor,
                                                                         MCAPLogLevel.INFO,
                                                                         Color.CORNFLOWERBLUE,
                                                                         MCAPLogLevel.WARNING,
                                                                         Color.ORANGE,
                                                                         MCAPLogLevel.ERROR,
                                                                         Color.RED,
                                                                         MCAPLogLevel.FATAL,
                                                                         Color.DARKRED);

      private final Color futureColor = Color.GRAY.deriveColor(0, 1, 1, 0.5);
      private final Map<MCAPLogLevel, String> logLevelToStringMap = Map.of(MCAPLogLevel.UNKNOWN,
                                                                           "  ???",
                                                                           MCAPLogLevel.DEBUG,
                                                                           "DEBUG",
                                                                           MCAPLogLevel.INFO,
                                                                           " INFO",
                                                                           MCAPLogLevel.WARNING,
                                                                           " WARN",
                                                                           MCAPLogLevel.ERROR,
                                                                           "ERROR",
                                                                           MCAPLogLevel.FATAL,
                                                                           "FATAL");
      private final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("HH:mm:ss.SSS a z");
      private final ZoneId zoneId = ZoneId.systemDefault(); // Need to parameterize this.
      private final ListView<MCAPConsoleLogItem> owner;
      private final YoLong currentTimestamp;
      private YoVariableChangedListener timestampListener;

      public MCAPConsoleLogItemListCell(ListView<MCAPConsoleLogItem> owner, MCAPLogSession session, YoLong currentTimestamp)
      {
         this.owner = owner;
         this.currentTimestamp = currentTimestamp;
      }

      @Override
      protected void updateItem(MCAPConsoleLogItem item, boolean empty)
      {
         super.updateItem(item, empty);

         if (empty || item == null)
         {
            setText(null);
            setGraphic(null);
            if (timestampListener != null)
               currentTimestamp.removeListener(timestampListener);
            timestampListener = null;
         }
         else
         {
            setFont(Font.font("Monospaced", 14.0));

            DoubleBinding cellWidthProperty = owner.widthProperty().subtract(15);
            minWidthProperty().bind(cellWidthProperty);
            prefWidthProperty().bind(cellWidthProperty);
            maxWidthProperty().bind(cellWidthProperty);

            setWrapText(true);
            updateTextFill();
            timestampListener = v -> listCellsToUpdate.add(this);
            currentTimestamp.addListener(timestampListener);
            String dateTimeFormatted = dateTimeFormatter.format(item.instant().atZone(zoneId));
            String logLevelString = logLevelToStringMap.get(item.logLevel() == null ? MCAPLogLevel.UNKNOWN : item.logLevel());
            setText("[%s] [%s]\n\t[%s]: %s".formatted(logLevelString, dateTimeFormatted, item.processName(), item.message()));
            setGraphic(null);
         }
      }

      private Paint previousTextFill;

      private void updateTextFill()
      {
         MCAPConsoleLogItem item = getItem();

         if (item == null)
         {
            previousTextFill = null;
            return;
         }

         Paint newTextFill;
         if (item.logTime() > currentTimestamp.getValue())
            newTextFill = futureColor;
         else if (logLevelToColorMap.containsKey(item.logLevel()))
            newTextFill = logLevelToColorMap.get(item.logLevel());
         else
            newTextFill = logLevelToColorMap.get(MCAPLogLevel.UNKNOWN);

         if (previousTextFill != newTextFill)
         {
            setTextFill(newTextFill);
            previousTextFill = newTextFill;
         }
      }
   }
}
