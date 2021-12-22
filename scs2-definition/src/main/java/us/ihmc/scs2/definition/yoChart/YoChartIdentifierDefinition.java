package us.ihmc.scs2.definition.yoChart;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoChartIdentifier")
public class YoChartIdentifierDefinition
{
   private int row;
   private int column;

   public YoChartIdentifierDefinition()
   {
   }

   public YoChartIdentifierDefinition(int row, int column)
   {
      setRow(row);
      setColumn(column);
   }

   @XmlAttribute
   public void setRow(int row)
   {
      this.row = row;
   }

   @XmlAttribute
   public void setColumn(int column)
   {
      this.column = column;
   }

   public int getRow()
   {
      return row;
   }

   public int getColumn()
   {
      return column;
   }

   @Override
   public YoChartIdentifierDefinition clone()
   {
      return new YoChartIdentifierDefinition(row, column);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoChartIdentifierDefinition)
      {
         YoChartIdentifierDefinition other = (YoChartIdentifierDefinition) object;
         if (row != other.row)
            return false;
         if (column != other.column)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "[row " + row + ", col " + column + "]";
   }
}