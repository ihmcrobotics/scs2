package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.yoChart.YoChartConfigurationDefinition;
import us.ihmc.scs2.definition.yoChart.YoChartGroupConfigurationDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.ChartIdentifier;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ChartTable2D
{
   private boolean ignoreSizePropertyListener = false;
   private final Property<ChartTable2DSize> size = new SimpleObjectProperty<>(new ChartTable2DSize(0, 0));
   private final Property<ChartTable2DSize> maxSize = new SimpleObjectProperty<>(new ChartTable2DSize(6, 6));
   private final Supplier<YoChartPanelController> chartBuilder;

   private final List<ChartChangeListener> listeners = new ArrayList<>();

   private YoChartPanelController[][] chartTable = new YoChartPanelController[0][0];

   public ChartTable2D(Supplier<YoChartPanelController> chartBuilder)
   {
      this.chartBuilder = chartBuilder;

      size.addListener((o, oldValue, newValue) ->
                       {
                          if (ignoreSizePropertyListener)
                             return;
                          resize(oldValue, newValue, false);
                       });
   }

   public ChartTable2DSize getSize()
   {
      return sizeProperty().getValue();
   }

   public Property<ChartTable2DSize> sizeProperty()
   {
      return size;
   }

   public ChartTable2DSize getMaxSize()
   {
      return maxSizeProperty().getValue();
   }

   public Property<ChartTable2DSize> maxSizeProperty()
   {
      return maxSize;
   }

   public boolean set(YoChartGroupConfigurationDefinition definition)
   {
      if (!maxSize.getValue().contains(definition.getNumberOfRows() - 1, definition.getNumberOfColumns() - 1))
      {
         LogTools.warn("Cannot set from configuration, required number of rows/columns is too large.");
         return false;
      }

      clear(); // TODO we can do without clearing and instead recycle charts if any.

      ChartTable2DSize newSize = new ChartTable2DSize(definition.getNumberOfRows(), definition.getNumberOfColumns());
      YoChartPanelController[][] newTable = new YoChartPanelController[newSize.getNumberOfRows()][newSize.getNumberOfCols()];
      YoChartConfigurationDefinition[][] newTableDefinition = extractChartDefinitionTable2D(definition);

      for (int row = 0; row < newSize.getNumberOfRows(); row++)
      {
         for (int col = 0; col < newSize.getNumberOfCols(); col++)
         {
            YoChartPanelController newChart = chartBuilder.get();
            newTable[row][col] = newChart;
            notifyChartAdded(newChart, row, col); // Listener first, so listener on chart plotted variables can be added.
            newChart.setChartConfiguration(newTableDefinition[row][col]);
         }
      }
      chartTable = newTable;
      ignoreSizePropertyListener = true;
      size.setValue(newSize);
      ignoreSizePropertyListener = false;
      return true;
   }

   private static YoChartConfigurationDefinition[][] extractChartDefinitionTable2D(YoChartGroupConfigurationDefinition definition)
   {
      YoChartConfigurationDefinition[][] tableDefinition = new YoChartConfigurationDefinition[definition.getNumberOfRows()][definition.getNumberOfColumns()];

      if (definition.getChartConfigurations() != null)
      {
         for (YoChartConfigurationDefinition chartDefinition : definition.getChartConfigurations())
         {
            tableDefinition[chartDefinition.getIdentifier().getRow()][chartDefinition.getIdentifier().getColumn()] = chartDefinition;
         }
      }

      return tableDefinition;
   }

   public void addListener(ChartChangeListener listener)
   {
      listeners.add(listener);
   }

   private void notifyChartAdded(YoChartPanelController chart, int row, int col)
   {
      ChartChange change = new ChartChange(ChangeType.ADD, chart, null, new ChartIdentifier(row, col));

      for (ChartChangeListener listener : listeners)
         listener.onChange(change);
   }

   private void notifyChartRemoved(YoChartPanelController chart, int row, int col)
   {
      if (chart == null)
         return;

      ChartChange change = new ChartChange(ChangeType.REMOVE, chart, new ChartIdentifier(row, col), null);

      for (ChartChangeListener listener : listeners)
         listener.onChange(change);
   }

   private void notifyChartMoved(YoChartPanelController chart, int fromRow, int fromCol, int toRow, int toCol)
   {
      if (chart == null)
         return;
      if (fromRow == toRow && fromCol == toCol)
         return; // Nothing moved really.

      ChartChange change = new ChartChange(ChangeType.MOVE, chart, new ChartIdentifier(fromRow, fromCol), new ChartIdentifier(toRow, toCol));

      for (ChartChangeListener listener : listeners)
         listener.onChange(change);
   }

   public void clear()
   {
      for (int row = 0; row < size.getValue().getNumberOfRows(); row++)
      {
         for (int col = 0; col < size.getValue().getNumberOfCols(); col++)
         {
            removeChart(row, col);
         }
      }
      chartTable = new YoChartPanelController[0][0];
      ignoreSizePropertyListener = true;
      size.setValue(new ChartTable2DSize(0, 0));
      ignoreSizePropertyListener = false;
   }

   public ChartTable2DSize resize(ChartTable2DSize desiredSize)
   {
      return resize(desiredSize, false);
   }

   private ChartTable2DSize resize(ChartTable2DSize desiredSize, boolean onlyConsiderNullForDownsize)
   {
      ChartTable2DSize newSize = resize(size.getValue(), desiredSize, onlyConsiderNullForDownsize);
      ignoreSizePropertyListener = true;
      size.setValue(newSize);
      ignoreSizePropertyListener = false;
      return newSize;
   }

   private ChartTable2DSize resize(ChartTable2DSize oldSize, ChartTable2DSize desiredSize, boolean onlyConsiderNullForDownsize)
   {
      if (chartTable.length != oldSize.getNumberOfRows() || (chartTable.length == 0 ? 0 : chartTable[0].length) != oldSize.getNumberOfCols())
         throw new IllegalStateException(String.format("Unexpected chartTable size: was [row=%d, col=%d], expected [row=%d, col=%d]",
                                                       chartTable.length,
                                                       chartTable.length == 0 ? 0 : chartTable[0].length,
                                                       oldSize.getNumberOfRows(),
                                                       oldSize.getNumberOfCols()));

      if (Objects.equals(oldSize, desiredSize))
         return desiredSize;

      if (!maxSize.getValue().contains(desiredSize))
      {
         LogTools.warn("Trying to set the size ({}) to be bigger than the max size ({}), reverting resize.", desiredSize, maxSize);
         return oldSize;
      }

      if (oldSize.isEmpty())
      { // Starting from a blank slate. Just pad with empty charts
         chartTable = new YoChartPanelController[desiredSize.getNumberOfRows()][desiredSize.getNumberOfCols()];

         for (int row = 0; row < desiredSize.getNumberOfRows(); row++)
         {
            for (int col = 0; col < desiredSize.getNumberOfCols(); col++)
            {
               notifyChartAdded(chartTable[row][col] = chartBuilder.get(), row, col);
            }
         }
      }
      else if (desiredSize.getNumberOfRows() >= oldSize.getNumberOfRows())
      {
         if (desiredSize.getNumberOfCols() > oldSize.getNumberOfCols())
         { // Only increasing, just pad new rows and columns with new charts.
            YoChartPanelController[][] newTable = new YoChartPanelController[desiredSize.getNumberOfRows()][desiredSize.getNumberOfCols()];

            for (int row = 0; row < desiredSize.getNumberOfRows(); row++)
            {
               for (int col = 0; col < desiredSize.getNumberOfCols(); col++)
               {
                  if (oldSize.contains(row, col)) // Copy the current table to the new one
                     newTable[row][col] = chartTable[row][col];
                  else // Pad the new rows
                     notifyChartAdded(newTable[row][col] = chartBuilder.get(), row, col);
               }
            }
            chartTable = newTable;
         }
         else
         { // Increasing rows and decreasing columns.
            // Figure out if it is possible to decrease the number of columns as desired.
            int minCols = computeMinColumns(oldSize, onlyConsiderNullForDownsize);
            if (minCols > desiredSize.getNumberOfCols())
            { // Can't reduce as much, doing best effort.
               desiredSize = new ChartTable2DSize(desiredSize.getNumberOfRows(), minCols);
               if (oldSize.equals(desiredSize))
                  return oldSize; // Nothing to do.
            }

            YoChartPanelController[][] newTable = new YoChartPanelController[desiredSize.getNumberOfRows()][desiredSize.getNumberOfCols()];

            // To keep track of the number of columns to be removed, so we can do a lazy removal.
            int colsToRemove = oldSize.getNumberOfCols() - desiredSize.getNumberOfCols();

            int newCol = desiredSize.getNumberOfCols() - 1;

            for (int oldCol = oldSize.getNumberOfCols() - 1; oldCol >= 0; oldCol--)
            {
               // Only remove the desired number of columns, not necessarily trying to reach min size.
               if (colsToRemove > 0 && isColumnEmpty(oldCol, oldSize, onlyConsiderNullForDownsize))
               { // We won't use these charts, notify listeners that they are removed.
                  colsToRemove--;
                  for (int row = 0; row < oldSize.getNumberOfRows(); row++)
                     notifyChartRemoved(chartTable[row][oldCol], row, oldCol);
                  continue;
               }

               for (int row = 0; row < desiredSize.getNumberOfRows(); row++)
               {
                  if (row < oldSize.getNumberOfRows()) // Shift the column
                     notifyChartMoved(newTable[row][newCol] = chartTable[row][oldCol], row, oldCol, row, newCol);
                  else // Pad the new rows with new empty charts
                     notifyChartAdded(newTable[row][newCol] = chartBuilder.get(), row, newCol);
               }
               newCol--;
            }
            chartTable = newTable;
         }
      }
      else if (desiredSize.getNumberOfCols() >= oldSize.getNumberOfCols())
      { // Increasing number of columns and decreasing number of rows.
         int minRows = computeMinRows(oldSize, onlyConsiderNullForDownsize);
         if (minRows > desiredSize.getNumberOfRows())
         { // Can't reduce as much, doing best effort.
            desiredSize = new ChartTable2DSize(minRows, desiredSize.getNumberOfCols());
            if (oldSize.equals(desiredSize))
               return oldSize; // Nothing to do.
         }

         YoChartPanelController[][] newTable = new YoChartPanelController[desiredSize.getNumberOfRows()][desiredSize.getNumberOfCols()];

         // To keep track of the number of rows to be removed, so we can do a lazy removal.
         int rowsToRemove = oldSize.getNumberOfRows() - desiredSize.getNumberOfRows();

         int newRow = desiredSize.getNumberOfRows() - 1;

         for (int oldRow = oldSize.getNumberOfRows() - 1; oldRow >= 0; oldRow--)
         {
            // Only remove the desired number of rows, not trying to reach min size.
            if (rowsToRemove > 0 && isRowEmpty(oldRow, oldSize, onlyConsiderNullForDownsize))
            { // We won't use these charts, notify listeners that they are removed.
               rowsToRemove--;
               for (int col = 0; col < oldSize.getNumberOfCols(); col++)
                  notifyChartRemoved(chartTable[oldRow][col], oldRow, col);
               continue;
            }

            for (int col = 0; col < desiredSize.getNumberOfCols(); col++)
            {
               if (col < oldSize.getNumberOfCols()) // Shift the row
                  notifyChartMoved(newTable[newRow][col] = chartTable[oldRow][col], oldRow, col, newRow, col);
               else // Pad with new charts
                  notifyChartAdded(newTable[newRow][col] = chartBuilder.get(), newRow, col);
            }
            newRow--;
         }
         chartTable = newTable;
      }
      else
      { // Decreasing both rows and columns
         int minRows = computeMinRows(oldSize, onlyConsiderNullForDownsize);
         int minCols = computeMinColumns(oldSize, onlyConsiderNullForDownsize);

         if (minRows > desiredSize.getNumberOfRows() || minCols > desiredSize.getNumberOfCols())
         { // Can't reduce as much, doing best effort.
            desiredSize = new ChartTable2DSize(minRows, minCols);
            if (oldSize.equals(desiredSize))
               return oldSize; // Nothing to do.
         }

         YoChartPanelController[][] newTable = new YoChartPanelController[desiredSize.getNumberOfRows()][desiredSize.getNumberOfCols()];

         // To keep track of the number of rows to be removed, so we can do a lazy removal.
         int rowsToRemove = oldSize.getNumberOfRows() - desiredSize.getNumberOfRows();

         int newRow = desiredSize.getNumberOfRows() - 1;

         for (int oldRow = oldSize.getNumberOfRows() - 1; oldRow >= 0; oldRow--)
         {
            if (rowsToRemove > 0 && isRowEmpty(oldRow, oldSize, onlyConsiderNullForDownsize))
            { // We won't use these charts, notify listeners that they are removed.
               rowsToRemove--;
               for (int oldCol = 0; oldCol < oldSize.getNumberOfCols(); oldCol++)
                  notifyChartRemoved(chartTable[oldRow][oldCol], oldRow, oldCol);
               continue;
            }

            // To keep track of the number of cols to be removed, so we can do a lazy removal.
            int colsToRemove = oldSize.getNumberOfCols() - desiredSize.getNumberOfCols();
            int newCol = desiredSize.getNumberOfCols() - 1;

            for (int oldCol = oldSize.getNumberOfCols() - 1; oldCol >= 0; oldCol--)
            {
               if (colsToRemove > 0 && isColumnEmpty(oldCol, oldSize, onlyConsiderNullForDownsize)) // TODO This can be optimized.
               { // We won't use this chart, notify listeners that it is removed.
                  colsToRemove--;
                  notifyChartRemoved(chartTable[oldRow][oldCol], oldRow, oldCol);
                  continue;
               }

               // Shift this chart in the grid
               notifyChartMoved(newTable[newRow][newCol] = chartTable[oldRow][oldCol], oldRow, oldCol, newRow, newCol);
               newCol--;
            }
            newRow--;
         }

         chartTable = newTable;
      }
      return desiredSize;
   }

   public void removeNullRowsAndColumns()
   {
      resize(new ChartTable2DSize(0, 0), true);
   }

   public YoChartPanelController get(int row, int col)
   {
      return chartTable[row][col];
   }

   public boolean isChartEmpty(int row, int col)
   {
      YoChartPanelController chart = chartTable[row][col];
      return chart == null || chart.isEmpty();
   }

   public boolean removeChart(YoChartPanelController chart)
   {
      for (int row = 0; row < size.getValue().getNumberOfRows(); row++)
      {
         for (int col = 0; col < size.getValue().getNumberOfCols(); col++)
         {
            if (chart == chartTable[row][col])
            {
               chartTable[row][col] = null;
               notifyChartRemoved(chart, row, col);
               return true;
            }
         }
      }
      return false;
   }

   public void removeChart(ChartIdentifier id)
   {
      removeChart(id.getRow(), id.getColumn());
   }

   public void removeChart(int row, int col)
   {
      YoChartPanelController chartToRemove = chartTable[row][col];
      chartTable[row][col] = null;
      notifyChartRemoved(chartToRemove, row, col);
   }

   public void removeEmptyCharts()
   {
      for (int row = 0; row < getSize().getNumberOfRows(); row++)
      {
         for (int col = 0; col < getSize().getNumberOfCols(); col++)
         {
            if (isChartEmpty(row, col))
               removeChart(row, col);
         }
      }
   }

   public ChartTable2DSize computeMinSize(ChartTable2DSize currentSize, boolean onlyNull)
   {
      return new ChartTable2DSize(computeMinRows(currentSize, onlyNull), computeMinColumns(currentSize, onlyNull));
   }

   private int computeMinRows(ChartTable2DSize currentSize, boolean onlyNull)
   {
      return currentSize.getNumberOfRows() - numberOfEmptyRows(currentSize, onlyNull);
   }

   private int computeMinColumns(ChartTable2DSize currentSize, boolean onlyNull)
   {
      return currentSize.getNumberOfCols() - numberOfEmptyColumns(currentSize, onlyNull);
   }

   private int numberOfEmptyRows(ChartTable2DSize size, boolean onlyNull)
   {
      int count = 0;
      for (int row = 0; row < size.getNumberOfRows(); row++)
      {
         if (isRowEmpty(row, size, onlyNull))
            count++;
      }
      return count;
   }

   private boolean isRowEmpty(int row, ChartTable2DSize size, boolean onlyNull)
   {
      if (row < 0 || row > size.getNumberOfRows())
         throw new IndexOutOfBoundsException(String.format("Row (%d) is out of bound, expected range [0,%d[", row, size.getNumberOfRows()));

      YoChartPanelController[] chartRow = chartTable[row];

      for (int col = 0; col < chartRow.length; col++)
      {
         YoChartPanelController chart = chartRow[col];
         if (onlyNull)
         {
            if (chart != null)
               return false;
         }
         else
         {
            if (chart != null && !chart.isEmpty())
               return false;
         }
      }

      return true;
   }

   private int numberOfEmptyColumns(ChartTable2DSize size, boolean onlyNull)
   {
      int count = 0;
      for (int col = 0; col < chartTable[0].length; col++)
      {
         if (isColumnEmpty(col, size, onlyNull))
            count++;
      }
      return count;
   }

   private boolean isColumnEmpty(int col, ChartTable2DSize size, boolean onlyNull)
   {
      if (col < 0 || col > size.getNumberOfCols())
         throw new IndexOutOfBoundsException(String.format("Col (%d) is out of bound, expected range [0,%d[", col, size.getNumberOfCols()));

      for (int row = 0; row < chartTable.length; row++)
      {
         YoChartPanelController chart = chartTable[row][col];
         if (onlyNull)
         {
            if (chart != null)
               return false;
         }
         else
         {
            if (chart != null && !chart.isEmpty())
               return false;
         }
      }
      return true;
   }

   public void forEachChart(Consumer<YoChartPanelController> action)
   {
      for (int row = 0; row < getSize().getNumberOfRows(); row++)
      {
         for (int col = 0; col < getSize().getNumberOfCols(); col++)
         {
            YoChartPanelController chart = get(row, col);
            if (chart != null)
               action.accept(chart);
         }
      }
   }

   public List<YoChartConfigurationDefinition> toChartDefinitions()
   {
      List<YoChartConfigurationDefinition> chartDefinitions = new ArrayList<>();
      for (int row = 0; row < getSize().getNumberOfRows(); row++)
      {
         for (int col = 0; col < getSize().getNumberOfCols(); col++)
         {
            if (!isChartEmpty(row, col))
               chartDefinitions.add(get(row, col).toYoChartConfigurationDefinition(new ChartIdentifier(row, col)));
         }
      }
      return chartDefinitions;
   }

   public static interface ChartChangeListener
   {
      void onChange(ChartChange c);
   }

   public enum ChangeType
   {
      ADD, REMOVE, MOVE
   }

   public static class ChartChange
   {
      private final ChangeType type;
      private final YoChartPanelController chart;
      private final ChartIdentifier from, to;

      private ChartChange(ChangeType type, YoChartPanelController chart, ChartIdentifier from, ChartIdentifier to)
      {
         this.type = type;
         this.chart = chart;
         this.from = from;
         this.to = to;
      }

      public ChangeType type()
      {
         return type;
      }

      public YoChartPanelController getChart()
      {
         return chart;
      }

      public ChartIdentifier from()
      {
         return from;
      }

      public int fromRow()
      {
         return from == null ? -1 : from.getRow();
      }

      public int fromCol()
      {
         return from == null ? -1 : from.getColumn();
      }

      public ChartIdentifier to()
      {
         return to;
      }

      public int toRow()
      {
         return to == null ? -1 : to.getRow();
      }

      public int toCol()
      {
         return to == null ? -1 : to.getColumn();
      }

      @Override
      public String toString()
      {
         switch (type)
         {
            case ADD:
               return String.format("Add    - [row=%d, col=%d]", toRow(), toCol());
            case REMOVE:
               return String.format("Remove - [row=%d, col=%d]", fromRow(), fromCol());
            case MOVE:
               return String.format("Move   - [row=%d, col=%d] => [row=%d, col=%d]", fromRow(), fromCol(), toRow(), toCol());
            default:
               throw new IllegalArgumentException("Unexpected value: " + type);
         }
      }
   }

   public static class ChartTable2DSize
   {
      private final int numberOfRows;
      private final int numberOfCols;

      public ChartTable2DSize(int numberOfRows, int numberOfCols)
      {
         this.numberOfRows = numberOfRows;
         this.numberOfCols = numberOfCols;
      }

      public int getNumberOfRows()
      {
         return numberOfRows;
      }

      public int getNumberOfCols()
      {
         return numberOfCols;
      }

      public boolean isEmpty()
      {
         return numberOfRows == 0 && numberOfCols == 0;
      }

      public boolean contains(int row, int col)
      {
         return row < numberOfRows && col < numberOfCols;
      }

      public boolean contains(ChartTable2DSize other)
      {
         return contains(other.numberOfRows - 1, other.numberOfCols - 1);
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
            return true;
         else if (object instanceof ChartTable2DSize other)
            return numberOfRows == other.numberOfRows && numberOfCols == other.numberOfCols;
         else
            return false;
      }

      @Override
      public String toString()
      {
         return String.format("Size: [nRows=%d, nCols=%d]", numberOfRows, numberOfCols);
      }
   }
}
