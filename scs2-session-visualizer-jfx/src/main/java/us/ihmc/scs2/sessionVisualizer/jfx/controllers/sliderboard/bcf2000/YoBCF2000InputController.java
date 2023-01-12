package us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.bcf2000;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Predicate;

import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.Node;
import javafx.scene.control.ContextMenu;
import javafx.scene.control.Labeled;
import javafx.scene.control.MenuItem;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.Region;
import javafx.scene.text.Text;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.sliderboard.YoVariableSlider;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.YoCompositeSearchManager;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.DragAndDropTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.NumberFormatTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class YoBCF2000InputController
{
   private static final String HIGHLIGHTED_BORDER = "-fx-border-color:green; -fx-border-radius:5;";
   private static final String HIGHLIGHTED_BACKGROUND = "-fx-background-color: #c5fcee88;";
   private static final String DEFAULT_BORDER = null;
   private static final String DEFAULT_BACKGROUND = null;

   private YoCompositeSearchManager yoCompositeSearchManager;

   private Region rootPane;
   private Labeled yoVariableDropLabel;

   private final SimpleObjectProperty<ContextMenu> contextMenuProperty = new SimpleObjectProperty<>(this, "buttonContextMenu", null);

   private final StringProperty backgroundStyle = new SimpleStringProperty(this, "backgroundStyle", DEFAULT_BACKGROUND);
   private final StringProperty borderStyle = new SimpleStringProperty(this, "borderStyle", DEFAULT_BORDER);
   private YoVariableSlider yoVariableSlider = null;

   private JavaFXMessager messager;
   private Topic<List<String>> yoCompositeSelectedTopic;
   private AtomicReference<List<String>> yoCompositeSelected;
   private Predicate<YoVariable> filter;
   private String defaultText = "Drop YoVariable here";

   public YoBCF2000InputController()
   {
   }

   protected void initialize(SessionVisualizerToolkit toolkit, Region rootPane, Labeled yoVariableDropLabel)
   {
      initialize(toolkit, rootPane, yoVariableDropLabel, var -> true);
   }

   protected void initialize(SessionVisualizerToolkit toolkit, Region rootPane, Labeled yoVariableDropLabel, Predicate<YoVariable> filter)
   {
      this.rootPane = rootPane;
      this.yoVariableDropLabel = yoVariableDropLabel;
      this.filter = filter;
      this.yoCompositeSearchManager = toolkit.getYoCompositeSearchManager();

      if (yoVariableDropLabel.getText() != null && !yoVariableDropLabel.getText().isEmpty())
         defaultText = yoVariableDropLabel.getText();

      ChangeListener<Object> styleChangeListener = (o, oldValue, newValue) ->
      { // TODO Consider switching to CSS
         String style = null;
         if (backgroundStyle.get() != null)
            style = backgroundStyle.get();
         if (borderStyle.get() != null)
            style = (style != null ? style + borderStyle.get() : borderStyle.get());
         rootPane.setStyle(style);
      };

      backgroundStyle.addListener(styleChangeListener);
      borderStyle.addListener(styleChangeListener);

      contextMenuProperty.addListener((ChangeListener<ContextMenu>) (observable, oldValue, newValue) ->
      {
         if (oldValue != null)
            oldValue.hide();
      });

      messager = toolkit.getMessager();
      yoCompositeSelectedTopic = toolkit.getTopics().getYoCompositeSelected();
      yoCompositeSelected = messager.createInput(yoCompositeSelectedTopic);

      rootPane.setOnDragDetected(this::handleDragDetected);
      rootPane.setOnDragOver(this::handleDragOver);
      rootPane.setOnDragDropped(this::handleDragDropped);
      rootPane.setOnDragEntered(this::handleDragEntered);
      rootPane.setOnDragExited(this::handleDragExited);
      rootPane.setOnMousePressed(this::handleMousePressed);
      rootPane.setOnMouseReleased(this::handleMouseReleased);
      yoVariableDropLabel.setOnMousePressed(this::handleMousePressed);
      yoVariableDropLabel.setOnMouseReleased(this::handleMouseReleased);
   }

   public abstract void setYoVariableInput(YoVariable yoVariable);

   protected boolean isMinValid(YoVariable yoVariable, String minStringValue)
   {
      if (minStringValue == null || minStringValue.isBlank())
         return false;

      if (yoVariable instanceof YoDouble yoDouble)
      {
         Double minValue = NumberFormatTools.parseDouble(minStringValue);
         if (minValue == null)
            return false;
         return minValue <= yoDouble.getValue();
      }

      try
      {
         if (yoVariable instanceof YoInteger yoInteger)
            return Integer.parseInt(minStringValue) <= yoInteger.getValue();

         if (yoVariable instanceof YoLong yoLong)
            return Long.parseLong(minStringValue) <= yoLong.getValue();
      }
      catch (NumberFormatException e)
      {
         return false;
      }

      return false;
   }

   protected boolean isMaxValid(YoVariable yoVariable, String maxStringValue)
   {
      if (maxStringValue == null || maxStringValue.isBlank())
         return false;

      if (yoVariable instanceof YoDouble yoDouble)
      {
         Double maxValue = NumberFormatTools.parseDouble(maxStringValue);
         if (maxValue == null)
            return false;
         return maxValue >= yoDouble.getValue();
      }

      try
      {
         if (yoVariable instanceof YoInteger yoInteger)
            return Integer.parseInt(maxStringValue) >= yoInteger.getValue();

         if (yoVariable instanceof YoLong yoLong)
            return Long.parseLong(maxStringValue) >= yoLong.getValue();
      }
      catch (NumberFormatException e)
      {
         return false;
      }

      return false;
   }

   public boolean isEmpty()
   {
      return yoVariableSlider == null;
   }

   protected void setupYoVariableSlider(YoVariableSlider yoVariableSlider)
   {
      this.yoVariableSlider = yoVariableSlider;
      backgroundStyle.set(HIGHLIGHTED_BACKGROUND);
      yoVariableDropLabel.setText(yoVariableSlider.getYoVariable().getName());
   }

   public void clear()
   {
      yoVariableSlider = null;
      backgroundStyle.set(DEFAULT_BACKGROUND);
      yoVariableDropLabel.setText(defaultText);
   }

   private void handleMousePressed(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY)
      {
         hideContextMenu();
      }
   }

   private void handleMouseReleased(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY)
      {
         if (yoVariableSlider != null && event.isStillSincePress())
         {
            messager.submitMessage(yoCompositeSelectedTopic, Arrays.asList(YoCompositeTools.YO_VARIABLE, yoVariableSlider.getYoVariable().getFullNameString()));
         }
      }
      else if (event.getButton() == MouseButton.SECONDARY)
      {
         if (yoVariableSlider != null && event.isStillSincePress())
         {
            ContextMenu contextMenu = newGraphContextMenu();
            if (!contextMenu.getItems().isEmpty())
            {
               contextMenuProperty.set(contextMenu);
               contextMenu.show(rootPane, event.getScreenX(), event.getScreenY());
            }
            event.consume();
         }
      }
      else if (event.getButton() == MouseButton.MIDDLE)
      {
         if (yoCompositeSelected.get() != null)
         {
            String type = yoCompositeSelected.get().get(0);
            if (type.equals(YoCompositeTools.YO_VARIABLE))
            {
               String fullname = yoCompositeSelected.get().get(1);
               YoComposite yoComposite = yoCompositeSearchManager.getYoComposite(type, fullname);

               if (yoComposite != null && filter.test(yoComposite.getYoComponents().get(0)))
               {
                  setYoVariableInput(yoComposite.getYoComponents().get(0));
                  messager.submitMessage(yoCompositeSelectedTopic, null);
               }
            }
         }
      }
   }

   private void hideContextMenu()
   {
      if (contextMenuProperty.get() != null)
         contextMenuProperty.set(null);
   }

   private ContextMenu newGraphContextMenu()
   {
      if (yoVariableSlider == null)
         return null;

      ContextMenu contextMenu = new ContextMenu();
      MenuItem menuItem = new MenuItem("Remove " + yoVariableSlider.getYoVariable().getName());
      menuItem.setMnemonicParsing(false);
      menuItem.setOnAction(e -> setYoVariableInput(null));
      contextMenu.getItems().add(menuItem);
      return contextMenu;
   }

   private void handleDragDetected(MouseEvent event)
   {
      if (event == null || yoVariableSlider == null)
         return;

      if (!event.isPrimaryButtonDown())
         return;

      PickResult pickResult = event.getPickResult();

      if (pickResult == null)
         return;

      Node intersectedNode = pickResult.getIntersectedNode();

      if (intersectedNode == null)
         return;

      YoVariable yoVariable = yoVariableSlider.getYoVariable();

      if (intersectedNode instanceof Text)
      {
         Text legend = (Text) intersectedNode;
         String yoVariableName = legend.getText().split("\\s+")[0];
         if (!yoVariableName.equals(yoVariable.getName()))
            return;
         Dragboard dragBoard = legend.startDragAndDrop(TransferMode.COPY);
         ClipboardContent clipboardContent = new ClipboardContent();
         clipboardContent.put(DragAndDropTools.YO_COMPOSITE_REFERENCE, Arrays.asList(YoCompositeTools.YO_VARIABLE, yoVariable.getFullNameString()));
         dragBoard.setContent(clipboardContent);
      }

      event.consume();
   }

   private void handleDragEntered(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
         setSelectionHighlight(true);
      event.consume();
   }

   private void handleDragExited(DragEvent event)
   {
      if (acceptDragEventForDrop(event))
         setSelectionHighlight(false);
      event.consume();
   }

   private void handleDragOver(DragEvent event)
   {
      if (!event.isAccepted() && acceptDragEventForDrop(event))
         event.acceptTransferModes(TransferMode.ANY);
      event.consume();
   }

   private void handleDragDropped(DragEvent event)
   {
      // TODO
      //      if (event.isAccepted())
      //         return;

      Dragboard db = event.getDragboard();
      boolean success = false;
      List<YoComposite> yoComposites = DragAndDropTools.retrieveYoCompositesFromDragBoard(db, yoCompositeSearchManager);
      if (yoComposites != null)
      {
         // TODO
         //         for (YoComposite yoComposite : yoComposites)
         setYoVariableInput(yoComposites.get(0).getYoComponents().get(0));
         success = true;
      }
      event.setDropCompleted(success);
      event.consume();

      if (success)
         setSelectionHighlight(false);
   }

   private boolean acceptDragEventForDrop(DragEvent event)
   {
      if (event.getGestureSource() == yoVariableDropLabel)
         return false;

      Dragboard dragboard = event.getDragboard();
      List<YoComposite> result = DragAndDropTools.retrieveYoCompositesFromDragBoard(dragboard, yoCompositeSearchManager);
      if (result == null || result.isEmpty())
         return false;
      if (result.get(0).getYoComponents().isEmpty())
         return false;
      if (yoVariableSlider != null && result.get(0).getYoComponents().get(0) == yoVariableSlider.getYoVariable())
         return false;
      return filter.test(result.get(0).getYoComponents().get(0));
   }

   private void setSelectionHighlight(boolean isSelected)
   {
      borderStyle.set(isSelected ? HIGHLIGHTED_BORDER : DEFAULT_BORDER);
   }
}
