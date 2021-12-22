package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic;

import javafx.beans.property.ReadOnlyBooleanProperty;
import javafx.beans.value.ObservableBooleanValue;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.UIElement;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;

public interface YoGraphicFXCreatorController<G extends YoGraphicFXItem> extends UIElement
{
   void initialize(SessionVisualizerToolkit toolkit, G yoGraphicToEdit);

   ObservableBooleanValue inputsValidityProperty();

   void resetFields();

   void saveChanges();

   ReadOnlyBooleanProperty hasChangesPendingProperty();

   default boolean hasChangesPending()
   {
      return hasChangesPendingProperty().get();
   }

   G getYoGraphicFX();
}
