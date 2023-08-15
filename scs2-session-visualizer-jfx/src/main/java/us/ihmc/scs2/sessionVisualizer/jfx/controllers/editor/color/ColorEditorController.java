package us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.color;

import java.io.IOException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableBooleanValue;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.ComboBox;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBADoubleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBAIntDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;

public class ColorEditorController
{
   @FXML
   private VBox mainPane;

   @FXML
   private ComboBox<ColorType> colorTypeComboBox;

   private enum ColorType
   {
      Simple, Yo_Red_Green_Blue_Double, Yo_Red_Green_Blue_Int, Yo_RGBA;

      @Override
      public String toString()
      {
         return name().replace('_', ' ');
      }
   };

   private final Map<ColorType, PaintEditorController<?>> paintEditors = new HashMap<>();
   private final ObjectProperty<BaseColorFX> colorProperty = new SimpleObjectProperty<>(this, "color", null);
   private final BooleanProperty inputsValidityProperty = new SimpleBooleanProperty(this, "inputsValidity", true);

   public void initialize(SessionVisualizerToolkit toolkit)
   {
      colorTypeComboBox.setItems(FXCollections.observableArrayList(ColorType.values()));
      colorTypeComboBox.getSelectionModel().select(null);

      colorTypeComboBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
      {
         if (oldValue != null)
         {
            PaintEditorController<?> oldEditor = paintEditors.get(oldValue);
            if (oldEditor != null)
               mainPane.getChildren().remove(oldEditor.getMainPane());
            colorProperty.unbind();
         }
         if (newValue != null)
         {
            PaintEditorController<?> newEditor = paintEditors.get(newValue);
            if (newEditor == null)
            {
               try
               {
                  FXMLLoader loader = new FXMLLoader(editorFXMLResource(newValue));
                  newEditor = newEditorController(newValue);
                  loader.setController(newEditor);
                  loader.load();
                  newEditor.initialize(toolkit);
                  paintEditors.put(newValue, newEditor);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
                  colorTypeComboBox.getSelectionModel().select(oldValue);
                  return;
               }
            }

            mainPane.getChildren().add(newEditor.getMainPane());
            colorProperty.bind(newEditor.colorProperty());
            inputsValidityProperty.bind(newEditor.inputsValidityProperty());
         }
      });
      colorTypeComboBox.getSelectionModel().select(ColorType.Simple);
   }

   public void setInput(PaintDefinition definition)
   {
      ColorType colorType = fromDefinition(definition);
      colorTypeComboBox.getSelectionModel().select(colorType);
      paintEditors.get(colorType).setInput(definition);
   }

   public ObservableBooleanValue inputsValidityProperty()
   {
      return inputsValidityProperty;
   }

   public ReadOnlyObjectProperty<BaseColorFX> colorProperty()
   {
      return colorProperty;
   }

   private static PaintEditorController<?> newEditorController(ColorType colorType)
   {
      switch (colorType)
      {
         case Simple:
            return new SimpleColorEditorController();
         case Yo_Red_Green_Blue_Double:
            return new YoColorRGBADoubleEditorController();
         case Yo_Red_Green_Blue_Int:
            return new YoColorRGBAIntEditorController();
         case Yo_RGBA:
            return new YoColorRGBASingleEditorController();
         default:
            LogTools.error("Unexpected value: {}", colorType);
            return new SimpleColorEditorController();
      }
   }

   private static URL editorFXMLResource(ColorType colorType)
   {
      switch (colorType)
      {
         case Simple:
            return SessionVisualizerIOTools.SIMPLE_COLOR_EDITOR_PANE_URL;
         case Yo_Red_Green_Blue_Double:
         case Yo_Red_Green_Blue_Int:
            return SessionVisualizerIOTools.YO_COLOR_RGBA_EDITOR_PANE_URL;
         case Yo_RGBA:
            return SessionVisualizerIOTools.YO_COLOR_RGBA_SINGLE_EDITOR_PANE_URL;
         default:
            LogTools.error("Unexpected value: {}", colorType);
            return SessionVisualizerIOTools.SIMPLE_COLOR_EDITOR_PANE_URL;
      }
   }

   private static ColorType fromDefinition(PaintDefinition definition)
   {
      if (definition == null)
         return ColorType.Simple;
      if (definition instanceof ColorDefinition)
         return ColorType.Simple;
      if (definition instanceof YoColorRGBADoubleDefinition)
         return ColorType.Yo_Red_Green_Blue_Double;
      if (definition instanceof YoColorRGBAIntDefinition)
         return ColorType.Yo_Red_Green_Blue_Int;
      if (definition instanceof YoColorRGBASingleDefinition)
         return ColorType.Yo_RGBA;
      LogTools.error("Unexpected color definition: {}", definition);
      return ColorType.Simple;
   }

   public Pane getMainPane()
   {
      return mainPane;
   }
}
