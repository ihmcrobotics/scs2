package us.ihmc.scs2.sessionVisualizer.jfx.controllers.menu;

import org.apache.commons.lang3.mutable.MutableBoolean;

import javafx.application.Platform;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.CheckMenuItem;
import javafx.scene.control.CustomMenuItem;
import javafx.scene.control.Menu;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.util.converter.IntegerStringConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.messager.SCS2Messager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionProperties;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.FXCoalescedUpdater;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sharedMemory.CropBufferRequest;
import us.ihmc.scs2.sharedMemory.FillBufferRequest;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;

import java.util.function.Consumer;

public class DataBufferMenuController implements VisualizerController
{
   @FXML
   private Menu menu;
   @FXML
   private CustomMenuItem bufferSizeMenuItem, bufferRecordTickPeriodMenuItem, numberPrecisionMenuItem;
   @FXML
   private TextField bufferSizeTextField;
   @FXML
   private TextField bufferRecordTickPeriodTextField;
   @FXML
   private Spinner<Integer> numberPrecisionSpinner;
   @FXML
   private CheckMenuItem enableFuzzyYoSearchMenuItem;
   @FXML
   private CheckMenuItem showSCS2YoVariablesMenuItem;

   private SCS2Messager messager;
   private SessionVisualizerTopics topics;
   private SessionVisualizerWindowToolkit toolkit;

   private boolean initializeBufferSizeTextField = true;
   private final Property<YoBufferPropertiesReadOnly> bufferProperties = new SimpleObjectProperty<>(this, "bufferProperties", null);
   private Consumer<YoBufferPropertiesReadOnly> bufferPropertiesListener;
   private final FXCoalescedUpdater<YoBufferPropertiesReadOnly> bufferPropertiesUpdater = new FXCoalescedUpdater<>(bufferProperties::setValue);
   private Consumer<SessionProperties> recordTickPeriodListener;
   /** Guards against feedback loops when a session-driven update sets a UI control's value. */
   private boolean updatingFromSession = false;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      this.toolkit = toolkit;
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();
      messager.addFXTopicListener(topics.getDisableUserControls(), disable -> menu.setDisable(disable));

      TextFormatter<Integer> bufferSizeFormatter = new TextFormatter<>(new IntegerStringConverter(), 0, new PositiveIntegerValueFilter());
      bufferSizeTextField.setTextFormatter(bufferSizeFormatter);

      MenuTools.configureTextFieldForCustomMenuItem(bufferSizeMenuItem, bufferSizeTextField);
      MenuTools.configureTextFieldForCustomMenuItem(bufferRecordTickPeriodMenuItem, bufferRecordTickPeriodTextField);

      MutableBoolean updatingBufferResize = new MutableBoolean(false);

      bufferProperties.addListener((o, oldValue, newValue) ->
      {
         if (!initializeBufferSizeTextField && (oldValue == null || newValue.getSize() == oldValue.getSize()))
            return;

         if (updatingBufferResize.isFalse())
         {
            updatingBufferResize.setTrue();
            bufferSizeFormatter.setValue(newValue.getSize());
            initializeBufferSizeTextField = false;
            updatingBufferResize.setFalse();
         }
      });

      bufferSizeFormatter.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (bufferProperties.getValue() != null && bufferProperties.getValue().getSize() == newValue.intValue())
            return;

         if (updatingBufferResize.isFalse())
         {
            updatingBufferResize.setTrue();
            if (toolkit.getSession() != null)
               toolkit.getSession().submitBufferSizeRequest(newValue);
            updatingBufferResize.setFalse();
         }
      });

      TextFormatter<Integer> recordPeriodFormatter = new TextFormatter<>(new IntegerStringConverter(), 0, new PositiveIntegerValueFilter());
      bufferRecordTickPeriodTextField.setTextFormatter(recordPeriodFormatter);

      recordPeriodFormatter.valueProperty().addListener((o, oldValue, newValue) ->
      {
         if (!updatingFromSession && toolkit.getSession() != null)
            toolkit.getSession().setBufferRecordTickPeriod(newValue);
      });

      toolkit.addAndTriggerSessionChangedListener((previousSession, newSession) ->
      {
         if (previousSession != null)
         {
            previousSession.removeCurrentBufferPropertiesListener(bufferPropertiesListener);
            previousSession.removeSessionPropertiesListener(recordTickPeriodListener);
         }

         if (newSession == null)
         {
            bufferPropertiesListener = null;
            recordTickPeriodListener = null;
            initializeBufferSizeTextField = true;
            return;
         }

         bufferPropertiesListener = bufferPropertiesUpdater::update;
         newSession.addCurrentBufferPropertiesListener(bufferPropertiesListener);

         recordTickPeriodListener = properties -> Platform.runLater(() ->
         {
            updatingFromSession = true;
            try
            {
               recordPeriodFormatter.setValue(properties.getBufferRecordTickPeriod());
            }
            finally
            {
               updatingFromSession = false;
            }
         });
         newSession.addSessionPropertiesListener(recordTickPeriodListener);
         recordTickPeriodListener.accept(newSession.getSessionProperties());
      });

      IntegerSpinnerValueFactory numberPrecisionSpinnerValueFactory = new IntegerSpinnerValueFactory(1, 30, 3, 1);
      numberPrecisionSpinner.setValueFactory(numberPrecisionSpinnerValueFactory);
      if (numberPrecisionSpinner.isEditable())
      {
         numberPrecisionSpinner.focusedProperty().addListener((o, oldValue, newValue) ->
         {
            if (!newValue)
            { // Losing focus
              // Workaround: manually reset to the current value
               numberPrecisionSpinner.getEditor()
                                     .setText(numberPrecisionSpinnerValueFactory.getConverter().toString(numberPrecisionSpinnerValueFactory.getValue()));
            }
         });
      }
      messager.bindBidirectional(topics.getControlsNumberPrecision(), numberPrecisionSpinnerValueFactory.valueProperty(), false);
      messager.bindBidirectional(topics.getShowSCS2YoVariables(), showSCS2YoVariablesMenuItem.selectedProperty(), false);
      enableFuzzyYoSearchMenuItem.selectedProperty().bindBidirectional(toolkit.getYoManager().enableFuzzyYoSearchProperty());
   }

   @FXML
   private void requestCropDataBuffer()
   {
      if (bufferProperties.getValue() != null && toolkit.getSession() != null)
      {
         CropBufferRequest cropBufferRequest = new CropBufferRequest(bufferProperties.getValue().getInPoint(), bufferProperties.getValue().getOutPoint());
         toolkit.getSession().submitCropBufferRequest(cropBufferRequest);
      }
   }

   @FXML
   private void requestFlushDataBuffer()
   {
      YoBufferPropertiesReadOnly properties = bufferProperties.getValue();
      if (properties != null && toolkit.getSession() != null)
      {
         FillBufferRequest fillBufferRequest = new FillBufferRequest(false,
                                                                     SharedMemoryTools.increment(properties.getOutPoint(), 1, properties.getSize()),
                                                                     SharedMemoryTools.decrement(properties.getInPoint(), 1, properties.getSize()));
         toolkit.getSession().submitFillBufferRequest(fillBufferRequest);
      }
   }
}
