package us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sessionVisualizer.jfx.controllers.chart.ChartTable2D.ChartTable2DSize;

public class ChartTable2DTest
{
   @Test
   public void testDefaultMaxSizeMatchesSavedChartWindowCapacity()
   {
      ChartTable2D chartTable = new ChartTable2D(() -> null);

      assertEquals(new ChartTable2DSize(9, 6), chartTable.getMaxSize());
   }

   @Test
   public void testDefaultMaxSizeAllowsNineRows()
   {
      ChartTable2D chartTable = new ChartTable2D(() -> null);

      assertEquals(new ChartTable2DSize(9, 6), chartTable.resize(new ChartTable2DSize(9, 6)));
   }
}
