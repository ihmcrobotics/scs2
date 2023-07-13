package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group;

import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.getCommonString;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.getCommonValue;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.getField;
import static us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.group.YoGroupFXEditorTools.setField;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.stream.Collectors;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ReadOnlyBooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.collections.ObservableSet;
import javafx.collections.SetChangeListener;
import javafx.scene.control.TextField;
import javafx.scene.image.ImageView;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoCompositeEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphic3DStyleEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.YoGraphicNameEditorPaneController;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.editor.searchTextField.DoubleSearchField;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic.YoGraphicFXCreatorController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.color.BaseColorFX;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;

public abstract class YoGroupFXEditorController<T extends YoGraphicFX> implements YoGraphicFXCreatorController<YoGroupFX>
{
   protected YoGroupFX yoGraphicToEdit;
   protected final ObservableList<T> graphicChildren = FXCollections.observableArrayList();
   protected YoCompositeSearchManager yoCompositeSearchManager;
   protected BooleanProperty hasChangesPendingProperty = new SimpleBooleanProperty(this, "hasChangesPending", false);
   private SessionVisualizerToolkit toolkit;

   private final List<Runnable> resetActions = new ArrayList<>();

   @Override
   public void initialize(SessionVisualizerToolkit toolkit, YoGroupFX yoGraphicToEdit)
   {
      this.toolkit = toolkit;
      this.yoGraphicToEdit = yoGraphicToEdit;
      yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();

      Class<T> childrenCommonType = getChildrenCommonType();

      ObservableSet<? extends YoGraphicFX> yoGraphicFXSet;

      if (YoGraphicFX2D.class.isAssignableFrom(childrenCommonType))
         yoGraphicFXSet = yoGraphicToEdit.getYoGraphicFX2DSet();
      else
         yoGraphicFXSet = yoGraphicToEdit.getYoGraphicFX3DSet();

      graphicChildren.setAll(yoGraphicFXSet.stream().filter(childrenCommonType::isInstance).map(childrenCommonType::cast).collect(Collectors.toList()));
      yoGraphicFXSet.addListener((SetChangeListener<YoGraphicFX>) change ->
      {
         if (change.wasRemoved() && childrenCommonType.isInstance(change.getElementRemoved()))
            graphicChildren.remove(change.getElementRemoved());
         if (change.wasAdded() && childrenCommonType.isInstance(change.getElementAdded()))
            graphicChildren.remove(change.getElementAdded());
      });
   }

   protected void setupNameEditor(YoGraphicNameEditorPaneController nameEditorController)
   {
      nameEditorController.initialize(toolkit, yoGraphicToEdit);
      nameEditorController.getNameLabel().setText("Group name");
      nameEditorController.bindYoGraphicFXItem(yoGraphicToEdit);
      registerResetAction(() -> nameEditorController.setInput(yoGraphicToEdit.getName(), yoGraphicToEdit.getNamespace()));
   }

   protected void setupStyleEditor(YoGraphic3DStyleEditorPaneController styleEditorController)
   {
      if (!YoGraphicFX3D.class.isAssignableFrom(getChildrenCommonType()))
         throw new IllegalStateException("Cannot setup the 3D style editor for " + getChildrenCommonType().getSimpleName());

      styleEditorController.initialize(toolkit);
      styleEditorController.colorProperty()
                           .addListener((o, oldValue, newValue) -> setField(graphicChildren, (g, c) -> ((YoGraphicFX3D) g).setColor(c), newValue));

      registerResetAction(() ->
      {
         BaseColorFX initialColor = getCommonValue(getField(graphicChildren, g -> ((YoGraphicFX3D) g).getColor()));
         if (initialColor != null)
            styleEditorController.setInput(YoGraphicTools.toPaintDefinition(initialColor));
      });
   }

   protected DoubleSearchField setupDoublePropertyEditor(TextField textField,
                                                         ImageView validImageView,
                                                         BiConsumer<T, DoubleProperty> setter,
                                                         Function<T, DoubleProperty> getter)
   {
      LinkedYoRegistry linkedRootRegistry = toolkit.getYoManager().getLinkedRootRegistry();

      DoubleSearchField yoDoubleTextField = new DoubleSearchField(textField, yoCompositeSearchManager, linkedRootRegistry, validImageView);
      yoDoubleTextField.setupAutoCompletion();
      yoDoubleTextField.supplierProperty().addListener((o, oldValue, newValue) -> setField(graphicChildren, setter, newValue));
      registerResetAction(() -> textField.setText(getCommonString(yoCompositeSearchManager, getField(graphicChildren, getter))));

      return yoDoubleTextField;
   }

   protected void setupTuple3DEditor(YoCompositeEditorPaneController tuple3DEditorController,
                                     boolean setupReferenceFrameFields,
                                     String entryName,
                                     BiConsumer<T, Tuple3DProperty> setter,
                                     Function<T, Tuple3DProperty> getter)
   {

      tuple3DEditorController.initialize(toolkit, yoCompositeSearchManager.getYoTuple3DCollection(), setupReferenceFrameFields);
      tuple3DEditorController.setCompositeName(entryName);
      registerResetAction(() ->
      {
         Tuple3DProperty commonTuple3DProperty = YoGroupFXEditorTools.getCommonTuple3DProperty(yoCompositeSearchManager, getField(graphicChildren, getter));
         if (commonTuple3DProperty != null)
            tuple3DEditorController.setInput(commonTuple3DProperty);
      });
      if (setupReferenceFrameFields)
      {
         tuple3DEditorController.addInputListener((components, frame) ->
         {
            Tuple3DProperty newValue = new Tuple3DProperty(frame, components);
            setField(graphicChildren, setter, newValue);
         });
      }
      else
      {
         tuple3DEditorController.addInputListener((components) ->
         {
            Tuple3DProperty newValue = new Tuple3DProperty(components);
            setField(graphicChildren, setter, newValue);
         });
      }
   }

   @Override
   public void resetFields()
   {
      resetActions.forEach(Runnable::run);
   }

   protected void registerResetAction(Runnable resetAction)
   {
      resetActions.add(resetAction);
   }

   @Override
   public void saveChanges()
   {
   }

   @Override
   public ReadOnlyBooleanProperty hasChangesPendingProperty()
   {
      return hasChangesPendingProperty;
   }

   @Override
   public YoGroupFX getYoGraphicFX()
   {
      return yoGraphicToEdit;
   }

   public abstract Class<T> getChildrenCommonType();
}
