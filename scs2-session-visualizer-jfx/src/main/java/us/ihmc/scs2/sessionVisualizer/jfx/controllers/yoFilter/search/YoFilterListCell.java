package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoFilter.search;

import javafx.beans.property.Property;
import javafx.beans.property.ReadOnlyProperty;
import javafx.geometry.Pos;
import javafx.scene.control.ContentDisplay;
import javafx.scene.control.Control;
import javafx.scene.control.Label;
import javafx.scene.control.Labeled;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.Spinner;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Region;
import javafx.scene.layout.VBox;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.scs2.filtering.YoFilter;
import us.ihmc.scs2.sessionVisualizer.jfx.YoNameDisplay;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoComposite.search.YoDoubleSpinnerValueFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoManager;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoDoubleProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.properties.YoVariableProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ScientificDoubleStringConverter;
import us.ihmc.scs2.sharedMemory.LinkedYoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

// FIXME Need to manually do some cleanup when the cell is being updated.
public class YoFilterListCell extends ListCell<YoFilter>
{
   // TODO Need to make the controls resizable
   private static final double GRAPHIC_PREF_WIDTH = 110.0;

   private final YoManager yoManager;
   private final ListView<YoFilter> owner;

   private final ReadOnlyProperty<YoNameDisplay> nameDisplay;

   private YoFilter yoFilter;
   private Labeled yoFilterNameDisplay = this;
   private final Property<Integer> numberPrecision;

   private final List<YoVariableProperty<?, ?>> yoVariableProperties = new ArrayList<>();

   public YoFilterListCell(YoManager yoManager, ReadOnlyProperty<YoNameDisplay> nameDisplay, Property<Integer> numberPrecision, ListView<YoFilter> owner)
   {
      this.yoManager = yoManager;
      this.nameDisplay = nameDisplay;
      this.numberPrecision = numberPrecision;
      this.owner = owner;
      getStyleClass().add("yo-variable-list-cell");
   }

   @Override
   protected void updateItem(YoFilter yoFilter, boolean empty)
   {
      this.yoFilter = yoFilter;
      super.updateItem(yoFilter, empty);

      // Cleanup the properties: remove listeners and disable linked buffer
      yoVariableProperties.forEach(YoVariableProperty::dispose);
      yoVariableProperties.clear();

      prefWidthProperty().bind(owner.widthProperty().subtract(15.0));
      setMinWidth(100.0);

      if (empty || yoManager.getLinkedRootRegistry() == null)
      {
         setGraphic(null);
         setText(null);
         setTooltip(null);
         return;
      }

      YoVariable yoVariable = yoFilter.getFilteredVariable();

      Region yoVariableControl = createYoVariableControl(yoVariable, numberPrecision, yoManager.getLinkedRootRegistry());
      setGraphic(yoVariableControl);
      setContentDisplay(ContentDisplay.LEFT);
      setAlignment(Pos.CENTER_LEFT);
      setGraphicTextGap(5);
      yoFilterNameDisplay = this;


      updateYoFilterName(nameDisplay.getValue());
      nameDisplay.addListener((o, oldValue, newValue) -> updateYoFilterName(newValue));
      yoFilterNameDisplay.setTooltip(new Tooltip(yoFilter.getName() + "\n" + yoFilter.getNamespace()));
   }

   private void updateYoFilterName(YoNameDisplay nameDisplay)
   {
      if (yoFilterNameDisplay == null || yoFilter == null)
         return;

      yoFilterNameDisplay.setText(switch (nameDisplay)
                                     {
                                        case SHORT_NAME, UNIQUE_NAME, UNIQUE_SHORT_NAME -> yoFilter.getName();
                                        case FULL_NAME -> yoFilter.getFullname();
                                     });
   }


   @SuppressWarnings({"unchecked", "rawtypes"})
   public Region createYoVariableControl(YoVariable yoVariable, Property<Integer> numberPrecision, LinkedYoRegistry linkedRegistry)
   {
      if (yoVariable instanceof YoDouble)
         return createYoDoubleControl((YoDouble) yoVariable, numberPrecision, linkedRegistry);
      throw new UnsupportedOperationException("Unhandled YoVariable type: " + yoVariable.getClass().getSimpleName());
   }

   public Control createYoDoubleControl(YoDouble yoDouble, Property<Integer> numberPrecision, LinkedYoRegistry linkedRegistry)
   {
      YoDoubleProperty yoDoubleProperty = new YoDoubleProperty(yoDouble, this);
      yoDoubleProperty.setLinkedBuffer(isDisabled() ? null : linkedRegistry.linkYoVariable(yoDouble, yoDoubleProperty));
      disabledProperty().addListener((o, oldValue, newValue) -> yoDoubleProperty.setLinkedBuffer(newValue ?
                                                                                                       null :
                                                                                                       linkedRegistry.linkYoVariable(yoDouble,
                                                                                                                                     yoDoubleProperty)));
      yoVariableProperties.add(yoDoubleProperty);

      YoDoubleSpinnerValueFactory valueFactory = new YoDoubleSpinnerValueFactory(yoDoubleProperty.getValue());
      DoubleStringConverter rawDoubleStringConverter = new DoubleStringConverter();
      ScientificDoubleStringConverter scientificDoubleStringConverter = new ScientificDoubleStringConverter(numberPrecision);
      valueFactory.setConverter(scientificDoubleStringConverter);
      Spinner<Double> spinner = new Spinner<>(valueFactory);
      spinner.setPrefWidth(GRAPHIC_PREF_WIDTH);
      spinner.setEditable(true);
      spinner.focusedProperty().addListener((o, oldValue, newValue) ->
                                            {
                                               valueFactory.setConverter(newValue ? rawDoubleStringConverter : scientificDoubleStringConverter);

                                               // When gaining focus: we want to update the text to reflect the change of the converter.
                                               // When losing focus: the text may be inconsistent with the actual value: reset the text.
                                               spinner.getEditor().setText(valueFactory.getConverter().toString(valueFactory.getValue()));
                                            });
      yoDoubleProperty.bindDoubleProperty(spinner.getValueFactory().valueProperty());

      Tooltip tooltip = new Tooltip();
      tooltip.textProperty().bind(spinner.valueProperty().asString());
      spinner.setTooltip(tooltip);

      return spinner;
   }

}