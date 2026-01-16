package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.controlsfx.control.textfield.TextFields;
import us.ihmc.scs2.definition.yoChart.YoChartConfigurationDefinition;
import us.ihmc.scs2.session.log.LogDataReaderInterface;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Arrays;
import java.util.Collection;
import java.util.stream.Collectors;

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
   private Button searchButton;
   @FXML
   private Button closeButton;

   private Stage stage;
   private SessionVisualizerToolkit toolkit;
   private LogDataReaderInterface mainLogReader;
   private LogDataReaderInterface addedLogReader;

   public void initialize(SessionVisualizerToolkit toolkit, Stage stage, LogDataReaderInterface mainLogReader, LogDataReaderInterface addedLogReader)
   {
      this.toolkit = toolkit;
      this.stage = stage;
      this.mainLogReader = mainLogReader;
      this.addedLogReader = addedLogReader;

      setupSearchField(mainLogSearchField, mainLogSearchStackPane, mainLogBestMatchLabel, mainLogReader);
      setupSearchField(addedLogSearchField, addedLogSearchStackPane, addedLogBestMatchLabel, addedLogReader);

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

   private void setupSearchField(TextField textArea, StackPane stackPane, Label bestMatchLabel, LogDataReaderInterface logDataReader)
   {
//      Collection<String> variableNames = logDataReader.getYoVariablesList()
//                                                      .stream()
//                                                      .map(YoVariable::getFullNameString)
//                                                      .collect(Collectors.toList());

//      SearchFieldWithHint searchFieldWithHint = new SearchFieldWithHint(textArea, variableNames);
//      stackPane.getChildren().add(searchFieldWithHint.getHintLabel());

      textArea.textProperty().addListener((o, oldValue, newValue) ->
      {
         String match = findFirstMatch(newValue, logDataReader);
         bestMatchLabel.setText(match != null ? match : "N/A");
      });
   }

   @FXML
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

   private String findFirstMatch(String query, LogDataReaderInterface logDataReader)
   {
      if (query == null || query.isEmpty())
         return null;

      String finalQuery = query.replaceAll("\n", "").trim();
      if (finalQuery.isEmpty())
         return null;

//      return logDataReader.getYoVariablesList()
//                          .stream()
//                          .map(YoVariable::getFullNameString)
//                          .filter(name -> us.ihmc.scs2.sessionVisualizer.jfx.controllers.RegularExpression.check(name, finalQuery))
//                          .findFirst()
//                          .orElse(null);
      return logDataReader.getYoVariablesList()
                          .stream()
                          .map(YoVariable::getFullNameString)
                          .filter(name -> name.contains(finalQuery))
                          .findFirst()
                          .orElse(null);
   }

   @FXML
   public void close()
   {
      stage.close();
   }
}
