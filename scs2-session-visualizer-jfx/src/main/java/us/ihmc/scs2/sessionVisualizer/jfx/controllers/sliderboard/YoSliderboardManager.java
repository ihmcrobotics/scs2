package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.URL;
import java.util.List;

import javax.xml.bind.JAXBException;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXMLLoader;
import javafx.stage.Window;
import javafx.util.Pair;
import us.ihmc.log.LogTools;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.definition.yoSlider.YoButtonDefinition;
import us.ihmc.scs2.definition.yoSlider.YoKnobDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SCSGuiConfiguration;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.Manager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SecondaryWindowManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.xml.XMLTools;

public class YoSliderboardManager implements Manager
{
   private final Property<YoMultiSliderboardWindowController> behringerSliderboard = new SimpleObjectProperty<>(this, "behringerSliderboard", null);
   private final SessionVisualizerToolkit toolkit;
   private final SessionVisualizerTopics topics;
   private final JavaFXMessager messager;

   private YoSliderboardListDefinition initialConfiguration = null;

   public YoSliderboardManager(SessionVisualizerToolkit toolkit)
   {
      messager = toolkit.getMessager();
      topics = toolkit.getTopics();

      this.toolkit = toolkit;
   }

   public void saveSessionConfiguration(SCSGuiConfiguration configuration)
   {
      handleSaveRequest(configuration.getYoSliderboardConfigurationFile());
   }

   private final TopicListener<File> saveRequestListener = m -> handleSaveRequest(m);
   private final TopicListener<File> loadRequestListener = m -> handleLoadRequest(m);
   private final TopicListener<Boolean> clearAllRequestListener = m -> handleClearRequest(m);
   private final TopicListener<YoSliderboardListDefinition> setMultiRequestListener = m -> handleSetRequest(m);
   private final TopicListener<YoSliderboardDefinition> setSingleRequestListener = m -> handleSetRequest(m);
   private final TopicListener<String> removeRequestListener = m -> handleRemoveRequest(m);
   private final TopicListener<Pair<String, YoButtonDefinition>> setButtonRequestListener = m -> handleSetButtonRequest(m.getKey(), m.getValue());
   private final TopicListener<Pair<String, YoKnobDefinition>> setKnobRequestListener = m -> handleSetKnobRequest(m.getKey(), m.getValue());
   private final TopicListener<Pair<String, YoSliderDefinition>> setSliderRequestListener = m -> handleSetSliderRequest(m.getKey(), m.getValue());
   private final TopicListener<Pair<String, Integer>> clearButtonRequestListener = m -> handleClearButtonRequest(m.getKey(), m.getValue());
   private final TopicListener<Pair<String, Integer>> clearKnobRequestListener = m -> handleClearKnobRequest(m.getKey(), m.getValue());
   private final TopicListener<Pair<String, Integer>> clearSliderRequestListener = m -> handleClearSliderRequest(m.getKey(), m.getValue());

   @Override
   public void startSession(Session session)
   {
      initialConfiguration = null;

      messager.addFXTopicListener(topics.getYoMultiSliderboardSave(), saveRequestListener);
      messager.addFXTopicListener(topics.getYoMultiSliderboardLoad(), loadRequestListener);
      messager.addFXTopicListener(topics.getYoMultiSliderboardClearAll(), clearAllRequestListener);
      messager.addFXTopicListener(topics.getYoMultiSliderboardSet(), setMultiRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardSet(), setSingleRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardRemove(), removeRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardSetButton(), setButtonRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardSetKnob(), setKnobRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardSetSlider(), setSliderRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardClearButton(), clearButtonRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardClearKnob(), clearKnobRequestListener);
      messager.addFXTopicListener(topics.getYoSliderboardClearSlider(), clearSliderRequestListener);
   }

   @Override
   public void stopSession()
   {
      messager.removeFXTopicListener(topics.getYoMultiSliderboardSave(), saveRequestListener);
      messager.removeFXTopicListener(topics.getYoMultiSliderboardLoad(), loadRequestListener);
      messager.removeFXTopicListener(topics.getYoMultiSliderboardClearAll(), clearAllRequestListener);
      messager.removeFXTopicListener(topics.getYoMultiSliderboardSet(), setMultiRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardSet(), setSingleRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardRemove(), removeRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardSetButton(), setButtonRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardSetKnob(), setKnobRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardSetSlider(), setSliderRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardClearButton(), clearButtonRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardClearKnob(), clearKnobRequestListener);
      messager.removeFXTopicListener(topics.getYoSliderboardClearSlider(), clearSliderRequestListener);

      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().close();
         behringerSliderboard.setValue(null);
      }
   }

   @Override
   public boolean isSessionLoaded()
   {
      // TODO Should probably return something else here.
      return false;
   }

   private void handleSaveRequest(File file)
   {
      YoSliderboardListDefinition definitionToSave;
      if (behringerSliderboard.getValue() != null)
         definitionToSave = behringerSliderboard.getValue().toYoSliderboardListDefinition();
      else
         definitionToSave = initialConfiguration;

      if (definitionToSave != null)
      {
         try
         {
            XMLTools.saveYoSliderboardListDefinition(new FileOutputStream(file), definitionToSave);
         }
         catch (IOException | JAXBException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void handleLoadRequest(File file)
   {
      LogTools.info("Loading from file: " + file);

      try
      {
         handleSetRequest(XMLTools.loadYoSliderboardListDefinition(new FileInputStream(file)));
      }
      catch (IOException | JAXBException e)
      {
         e.printStackTrace();
      }
   }

   private void handleClearRequest(Boolean m)
   {
      if (behringerSliderboard.getValue() != null)
         behringerSliderboard.getValue().clear();
      else
         initialConfiguration = null;
   }

   private void handleSetRequest(YoSliderboardListDefinition definition)
   {
      if (behringerSliderboard.getValue() != null)
         behringerSliderboard.getValue().setInput(definition);
      else
         initialConfiguration = new YoSliderboardListDefinition(definition);
   }

   private void handleSetRequest(YoSliderboardDefinition definition)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().setSliderboard(definition);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int index = findSliderboardIndex(definition.getName());
         if (index != -1)
            initialConfiguration.getYoSliderboards().get(index).set(definition);
         else
            initialConfiguration.getYoSliderboards().add(definition);
      }
   }

   private void handleRemoveRequest(String sliderboardName)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().closeSliderboard(sliderboardName);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int index = findSliderboardIndex(sliderboardName);
         if (index != -1)
            initialConfiguration.getYoSliderboards().remove(index);
      }
   }

   private void handleSetButtonRequest(String sliderboardName, YoButtonDefinition buttonDefinition)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().setButtonInput(sliderboardName, buttonDefinition);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int sliderboardIndex = findSliderboardIndex(sliderboardName);
         YoSliderboardDefinition sliderboard;
         if (sliderboardIndex != -1)
         {
            sliderboard = initialConfiguration.getYoSliderboards().get(sliderboardIndex);
         }
         else
         {
            sliderboard = new YoSliderboardDefinition(sliderboardName);
            initialConfiguration.getYoSliderboards().add(sliderboard);
         }
         sliderboard.getButtons().add(new YoButtonDefinition(buttonDefinition));
      }
   }

   private void handleSetKnobRequest(String sliderboardName, YoKnobDefinition knobDefinition)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().setKnobInput(sliderboardName, knobDefinition);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int sliderboardIndex = findSliderboardIndex(sliderboardName);
         YoSliderboardDefinition sliderboard;
         if (sliderboardIndex != -1)
         {
            sliderboard = initialConfiguration.getYoSliderboards().get(sliderboardIndex);
         }
         else
         {
            sliderboard = new YoSliderboardDefinition(sliderboardName);
            initialConfiguration.getYoSliderboards().add(sliderboard);
         }
         sliderboard.getKnobs().add(new YoKnobDefinition(knobDefinition));
      }
   }

   private void handleSetSliderRequest(String sliderboardName, YoSliderDefinition sliderDefinition)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().setSliderInput(sliderboardName, sliderDefinition);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int sliderboardIndex = findSliderboardIndex(sliderboardName);
         YoSliderboardDefinition sliderboard;
         if (sliderboardIndex != -1)
         {
            sliderboard = initialConfiguration.getYoSliderboards().get(sliderboardIndex);
         }
         else
         {
            sliderboard = new YoSliderboardDefinition(sliderboardName);
            initialConfiguration.getYoSliderboards().add(sliderboard);
         }
         sliderboard.getSliders().add(new YoSliderDefinition(sliderDefinition));
      }
   }

   private void handleClearButtonRequest(String sliderboardName, int buttonIndex)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().removeButtonInput(sliderboardName, buttonIndex);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int sliderboardIndex = findSliderboardIndex(sliderboardName);

         if (sliderboardIndex == -1)
            return;

         YoSliderboardDefinition sliderboard = initialConfiguration.getYoSliderboards().get(sliderboardIndex);

         List<YoButtonDefinition> buttons = sliderboard.getButtons();
         for (int i = buttons.size() - 1; i >= 0; i--)
         {
            if (i == buttonIndex && buttons.get(i).getIndex() == -1)
               buttons.remove(i);
            else if (buttons.get(i).getIndex() == buttonIndex)
               buttons.remove(i);
         }
      }
   }

   private void handleClearKnobRequest(String sliderboardName, int knobIndex)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().removeKnobInput(sliderboardName, knobIndex);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int sliderboardIndex = findSliderboardIndex(sliderboardName);

         if (sliderboardIndex == -1)
            return;

         YoSliderboardDefinition sliderboard = initialConfiguration.getYoSliderboards().get(sliderboardIndex);

         List<YoKnobDefinition> knobs = sliderboard.getKnobs();
         for (int i = knobs.size() - 1; i >= 0; i--)
         {
            if (i == knobIndex && knobs.get(i).getIndex() == -1)
               knobs.remove(i);
            else if (knobs.get(i).getIndex() == knobIndex)
               knobs.remove(i);
         }
      }
   }

   private void handleClearSliderRequest(String sliderboardName, int sliderIndex)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().removeSliderInput(sliderboardName, sliderIndex);
      }
      else
      {
         if (initialConfiguration == null)
            initialConfiguration = new YoSliderboardListDefinition();

         int sliderboardIndex = findSliderboardIndex(sliderboardName);

         if (sliderboardIndex == -1)
            return;

         YoSliderboardDefinition sliderboard = initialConfiguration.getYoSliderboards().get(sliderboardIndex);

         List<YoSliderDefinition> sliders = sliderboard.getSliders();
         for (int i = sliders.size() - 1; i >= 0; i--)
         {
            if (i == sliderIndex && sliders.get(i).getIndex() == -1)
               sliders.remove(i);
            else if (sliders.get(i).getIndex() == sliderIndex)
               sliders.remove(i);
         }
      }
   }

   private int findSliderboardIndex(String sliderboardName)
   {
      if (sliderboardName == null)
         return -1;

      List<YoSliderboardDefinition> yoSliderboards = initialConfiguration.getYoSliderboards();

      for (int i = 0; i < yoSliderboards.size(); i++)
      {
         YoSliderboardDefinition sliderboard = yoSliderboards.get(i);
         if (sliderboardName.equals(sliderboard.getName()))
            return i;
      }
      return -1;
   }
   
   public void openSliderboardWindow(Window requestSource, URL initialSliderboardWindow)
   {
      if (behringerSliderboard.getValue() != null)
      {
         behringerSliderboard.getValue().showWindow();
         return;
      }

      try
      {
         FXMLLoader fxmlLoader = new FXMLLoader(initialSliderboardWindow);
         fxmlLoader.load();
         YoMultiSliderboardWindowController controller = fxmlLoader.getController();
         controller.initialize(toolkit);
         if (initialConfiguration != null)
         {
            controller.setInput(initialConfiguration);
            initialConfiguration = null;
         }
         behringerSliderboard.setValue(controller);
         SecondaryWindowManager.initializeSecondaryWindowWithOwner(requestSource, controller.getWindow());
         controller.showWindow();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void openBFC2000SliderboardWindow(Window requestSource)
   {
      openSliderboardWindow(requestSource, SessionVisualizerIOTools.YO_MULTI_SLIDERBOARD_BCF2000_WINDOW_URL);
      behringerSliderboard.getValue().ensureBFC2000Tab();
   }

   public void openXTouchCompactSliderboardWindow(Window requestSource)
   {
      openSliderboardWindow(requestSource, SessionVisualizerIOTools.YO_MULTI_SLIDERBOARD_XTOUCHCOMPACT_WINDOW_URL);
      behringerSliderboard.getValue().ensureXTouchCompactTab();
   }
}
