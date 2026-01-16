package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.scs2.definition.yoChart.YoChartConfigurationDefinition;
import us.ihmc.scs2.session.log.LogDataReaderInterface;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.session.BindSynchronizingVariablesRequest;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Arrays;

public class LogVariableComparisonSearchDialogController
{
   @FXML
   private TextField mainLogSearchField;
   @FXML
   private StackPane mainLogSearchStackPane;
   @FXML
   private TextField addedLogSearchField;
   @FXML
   private StackPane addedLogSearchStackPane;
   @FXML
   private Label mainLogBestMatchLabel;
   @FXML
   private Label addedLogBestMatchLabel;
   @FXML
   private Label mainLogBestMatchLabelShort;
   @FXML
   private Label addedLogBestMatchLabelShort;
   @FXML
   private Button selectButton;

   private Stage stage;
   private SessionVisualizerToolkit toolkit;
   private LogDataReaderInterface addedLogReader;

   public void initialize(SessionVisualizerToolkit toolkit, Stage stage, LogDataReaderInterface mainLogReader, LogDataReaderInterface addedLogReader)
   {
      this.toolkit = toolkit;
      this.stage = stage;
      this.addedLogReader = addedLogReader;

      setupSearchField(mainLogSearchField, mainLogBestMatchLabel, mainLogBestMatchLabelShort, mainLogReader);
      setupSearchField(addedLogSearchField, addedLogBestMatchLabel, addedLogBestMatchLabelShort, addedLogReader);

      mainLogSearchField.addEventFilter(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ENTER)
         {
            if (e.isShiftDown() || e.isControlDown())
            {
               mainLogSearchField.appendText("\n");
            }
            else
            {
               search();
            }
            e.consume();
         }
      });
      addedLogSearchField.addEventFilter(KeyEvent.KEY_PRESSED, e ->
      {
         if (e.getCode() == KeyCode.ENTER)
         {
            if (e.isShiftDown() || e.isControlDown())
            {
               addedLogSearchField.appendText("\n");
            }
            else
            {
               search();
            }
            e.consume();
         }
      });
   }

   private void setupSearchField(TextField textArea, Label bestMatchLabel, Label bestMatchLabelShort, LogDataReaderInterface logDataReader)
   {
      textArea.textProperty().addListener((o, oldValue, newValue) ->
      {
         YoVariable match = findFirstMatch(newValue, logDataReader);
         if (match != null)
         {
            bestMatchLabel.setText(match.getFullNameString());
            bestMatchLabelShort.setText(match.getName());
         }
         else
         {
            bestMatchLabel.setText("N/A");
            bestMatchLabelShort.setText("N/A");
         }
      });
   }

   public void search()
   {
      String mainLogVarName = mainLogSearchField.getText();
      String addedLogVarName = addedLogSearchField.getText();

      // If the user hasn't typed anything, we can't search.
      if (mainLogVarName == null || mainLogVarName.trim().isEmpty() || addedLogVarName == null || addedLogVarName.trim().isEmpty())
         return;

      mainLogVarName = mainLogBestMatchLabel.getText();
      addedLogVarName = addedLogBestMatchLabel.getText();

      if (mainLogVarName.equals("N/A") || addedLogVarName.equals("N/A"))
         return;

      YoChartConfigurationDefinition chartDefinition = new YoChartConfigurationDefinition(Arrays.asList(mainLogVarName, addedLogVarName));
      toolkit.getMessager().submitMessage(toolkit.getTopics().getYoChartListAdd(), new ImmutablePair<>("Log Comparison", chartDefinition));
   }

   private YoVariable findFirstMatch(String query, LogDataReaderInterface logDataReader)
   {
      if (query == null || query.isEmpty())
         return null;

      String finalQuery = query.replaceAll("\n", "").trim();
      if (finalQuery.isEmpty())
         return null;

      return logDataReader.getYoVariablesList().stream().filter(v -> v.getName().contains(finalQuery)).findFirst().orElse(null);
   }

   @FXML
   public void select()
   {
      String mainLogVarName = mainLogSearchField.getText();
      String addedLogVarName = addedLogSearchField.getText();

      // If the user hasn't typed anything, we can't search.
      if (mainLogVarName == null || mainLogVarName.trim().isEmpty() || addedLogVarName == null || addedLogVarName.trim().isEmpty())
         return;

      String bestMatchMainName  = mainLogBestMatchLabel.getText();
      String bestMatchAddedName = addedLogBestMatchLabel.getText();

      if (!bestMatchMainName.equals("N/A") && !bestMatchAddedName.equals("N/A"))
      {
         BindSynchronizingVariablesRequest request = new BindSynchronizingVariablesRequest(addedLogReader.getLogDirectory().getAbsolutePath(), mainLogVarName, addedLogVarName);
         toolkit.getMessager().submitMessage(toolkit.getTopics().getBindSynchronizingVariablesRequest(), request);
      }

      stage.close();
   }
}
