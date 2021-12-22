package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class ChartGroupModel
{
   private final String name;
   private final List<ChartIdentifier> chartIdentifiers;

   public ChartGroupModel(String name)
   {
      this(name, new ArrayList<>());
   }

   public ChartGroupModel(ChartGroupModel other)
   {
      this.name = other.name;
      this.chartIdentifiers = new ArrayList<>(other.chartIdentifiers);
   }

   public ChartGroupModel(String name, List<ChartIdentifier> chartIdentifiers)
   {
      this.name = name;
      this.chartIdentifiers = chartIdentifiers;
   }

   public void clear()
   {
      chartIdentifiers.clear();
   }

   public int size()
   {
      return chartIdentifiers.size();
   }

   public boolean add(ChartIdentifier chartIdentifier)
   {
      return chartIdentifiers.add(chartIdentifier);
   }

   public boolean remove(ChartIdentifier chartIdentifier)
   {
      return chartIdentifiers.remove(chartIdentifier);
   }

   public String getName()
   {
      return name;
   }

   public List<ChartIdentifier> getChartIdentifiers()
   {
      return chartIdentifiers;
   }

   public int rowStart()
   {
      return chartIdentifiers.stream().mapToInt(ChartIdentifier::getRow).min().orElse(-1);
   }

   public int rowEnd()
   {
      return chartIdentifiers.stream().mapToInt(ChartIdentifier::getRow).max().orElse(-1);
   }

   public int rowSpan()
   {
      return rowEnd() - rowStart();
   }

   public int columnStart()
   {
      return chartIdentifiers.stream().mapToInt(ChartIdentifier::getColumn).min().orElse(-1);
   }

   public int columnEnd()
   {
      return chartIdentifiers.stream().mapToInt(ChartIdentifier::getColumn).max().orElse(-1);
   }

   public int columnSpan()
   {
      return columnEnd() - columnStart();
   }

   public ChartGroupModel shift(int rowShift, int columnShift)
   {
      return new ChartGroupModel(name, chartIdentifiers.stream().map(id -> id.shift(rowShift, columnShift)).collect(Collectors.toList()));
   }

   @Override
   public int hashCode()
   {
      return chartIdentifiers.hashCode();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ChartGroupModel)
      {
         ChartGroupModel other = (ChartGroupModel) object;
         return chartIdentifiers.equals(other.chartIdentifiers);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return name + ": " + chartIdentifiers.toString();
   }
}
