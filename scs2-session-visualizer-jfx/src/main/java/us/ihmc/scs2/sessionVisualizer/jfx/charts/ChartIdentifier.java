package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import us.ihmc.scs2.definition.yoChart.YoChartIdentifierDefinition;

public class ChartIdentifier
{
   private final int row;
   private final int column;

   public ChartIdentifier(int row, int column)
   {
      this.row = row;
      this.column = column;
   }

   public ChartIdentifier(YoChartIdentifierDefinition definition)
   {
      this(definition.getRow(), definition.getColumn());
   }

   public int getRow()
   {
      return row;
   }

   public int getColumn()
   {
      return column;
   }

   public ChartIdentifier shift(int rowShift, int columnShift)
   {
      return new ChartIdentifier(row + rowShift, column + columnShift);
   }

   public YoChartIdentifierDefinition toYoChartIdentifierDefinition()
   {
      return new YoChartIdentifierDefinition(row, column);
   }

   private int hashCode = 0;

   @Override
   public int hashCode()
   {
      if (hashCode == 0)
         hashCode = 31 * row + column;
      return hashCode;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ChartIdentifier)
      {
         ChartIdentifier other = (ChartIdentifier) object;
         return row == other.row && column == other.column;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "Row " + row + ", column " + column;
   }
}
