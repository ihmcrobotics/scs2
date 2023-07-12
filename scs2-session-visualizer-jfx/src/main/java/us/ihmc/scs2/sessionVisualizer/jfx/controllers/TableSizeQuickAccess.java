package us.ihmc.scs2.sessionVisualizer.jfx.controllers;

import com.jfoenix.controls.JFXButton;

import javafx.beans.property.IntegerProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.scene.control.Button;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.StrokeLineCap;
import javafx.scene.shape.StrokeLineJoin;
import javafx.scene.shape.StrokeType;
import javafx.scene.text.Text;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.ChartTable2D.ChartTable2DSize;

public class TableSizeQuickAccess
{
   private static final String UNSELECTED = "-fx-stroke:black; -fx-effect: dropshadow(three-pass-box, rgba(0,0,0,0.8), 3, 0, 0, 0);";
   private static final String SELECTED = "-fx-stroke:darkred; -fx-effect: dropshadow(three-pass-box, rgba(0.3,0,0,0.8), 5, 0, 0, 0);";

   private final AnchorPane anchorPane = new AnchorPane();
   private final VBox vBox = new VBox(5.0);
   private final GridPane gridPane = new GridPane();
   private final Tooltip tooltip = new Tooltip();
   private final Rectangle[][] boxes;
   private final Button clearAllButton, clearEmptyButton;

   private final ChartTable2DSize maxSize;

   private final IntegerProperty selectedRowsProperty = new SimpleIntegerProperty(this, "selectedRows", -1);
   private final IntegerProperty selectedColumnsProperty = new SimpleIntegerProperty(this, "selectedColumns", -1);

   public TableSizeQuickAccess(String title, ChartTable2DSize currentSize, ChartTable2DSize maxSize)
   {
      this.maxSize = maxSize;
      anchorPane.getChildren().add(vBox);
      AnchorPane.setTopAnchor(vBox, 5.0);
      AnchorPane.setLeftAnchor(vBox, 5.0);
      AnchorPane.setRightAnchor(vBox, 5.0);
      AnchorPane.setBottomAnchor(vBox, 5.0);
      vBox.getChildren().add(gridPane);

      clearAllButton = new JFXButton("Clear All");
      clearAllButton.getStyleClass().add("chart-table-view-clear-button");

      clearEmptyButton = new JFXButton("Clear Empty");
      clearEmptyButton.getStyleClass().add("chart-table-view-clear-button");

      vBox.getChildren().add(new HBox(3.0, clearAllButton, clearEmptyButton));
      vBox.getStylesheets().add(SessionVisualizerIOTools.GENERAL_STYLESHEET.toExternalForm());

      if (title != null)
      {
         Text titleText = new Text(title);
         vBox.getChildren().add(0, titleText);
      }

      gridPane.setHgap(5.0);
      gridPane.setVgap(5.0);
      gridPane.setOnMouseExited(e ->
      {
         updateSelectionStyleHandler(-1, -1);
         tooltip.hide();
      });
      gridPane.setOnMouseMoved(e -> tooltip.show(gridPane, e.getScreenX() + 10, e.getScreenY() + 20));

      boxes = new Rectangle[maxSize.getNumberOfRows()][maxSize.getNumberOfCols()];

      for (int row = 0; row < maxSize.getNumberOfRows(); row++)
      {
         for (int col = 0; col < maxSize.getNumberOfCols(); col++)
         {
            Paint boxPaint;
            if (row < currentSize.getNumberOfRows() && col < currentSize.getNumberOfCols())
               boxPaint = Paint.valueOf("#b5b576"); // Light gray-ish yellow
            else
               boxPaint = Paint.valueOf("#a0a4a8"); // Light gray
            Rectangle box = new Rectangle(30, 30, boxPaint);
            box.setStroke(Color.BLACK);
            box.setStrokeWidth(2.0);
            box.setStrokeType(StrokeType.INSIDE);
            box.setStrokeLineCap(StrokeLineCap.SQUARE);
            box.setStrokeLineJoin(StrokeLineJoin.MITER);
            box.setStrokeMiterLimit(10.0);
            box.setStyle(UNSELECTED);
            boxes[row][col] = box;
            gridPane.getChildren().add(box);
            GridPane.setRowIndex(box, row);
            GridPane.setColumnIndex(box, col);

            int finalRow = row;
            int finalCol = col;

            box.setOnMouseEntered(e -> updateSelectionStyleHandler(finalRow, finalCol));
         }
      }
   }

   private void updateSelectionStyleHandler(int lastSelectedRow, int lastSelectedCol)
   {
      for (int row = 0; row <= lastSelectedRow; row++)
      {
         Rectangle[] rowBoxes = boxes[row];

         for (int col = 0; col <= lastSelectedCol; col++)
         {
            rowBoxes[col].setStyle(SELECTED);
         }

         for (int col = lastSelectedCol + 1; col < maxSize.getNumberOfCols(); col++)
         {
            rowBoxes[col].setStyle(UNSELECTED);
         }
      }

      for (int row = lastSelectedRow + 1; row < maxSize.getNumberOfRows(); row++)
      {
         for (int col = 0; col < maxSize.getNumberOfCols(); col++)
         {
            boxes[row][col].setStyle(UNSELECTED);
         }
      }

      selectedRowsProperty.set(lastSelectedRow + 1);
      selectedColumnsProperty.set(lastSelectedCol + 1);
      tooltip.setText(String.format("Rows: %d, columns: %d", selectedRowsProperty.get(), selectedColumnsProperty.get()));
   }

   public IntegerProperty selectedRowsProperty()
   {
      return selectedRowsProperty;
   }

   public IntegerProperty selectedColumnsProperty()
   {
      return selectedColumnsProperty;
   }

   public Button getClearAllButton()
   {
      return clearAllButton;
   }

   public Button getClearEmptyButton()
   {
      return clearEmptyButton;
   }

   public AnchorPane getMainPane()
   {
      return anchorPane;
   }
}
