package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class DouglasPeuckerReduction
{
   public static <T extends Point2DReadOnly> List<T> reduce(List<T> rawPoints, double epsilon)
   {
      if (rawPoints.size() <= 2)
         return rawPoints;

      // Find the point with the maximum distance
      double dmax = 0;
      int index = 0;
      int end = rawPoints.size();
      T firstPoint = rawPoints.get(0);
      T lastPoint = rawPoints.get(end - 1);

      for (int i = 1; i < end; i++)
      {
         double d = EuclidGeometryTools.distanceFromPoint2DToLine2D(rawPoints.get(i), firstPoint, lastPoint);
         if (d > dmax)
         {
            index = i;
            dmax = d;
         }
      }

      // If max distance is greater than epsilon, recursively simplify
      if (dmax > epsilon)
      {
         // Recursive call
         List<T> subListA = reduce(rawPoints.subList(0, index), epsilon);
         List<T> subListB = reduce(rawPoints.subList(index, end), epsilon);
         List<T> result = new ArrayList<>(subListA.size() + subListB.size());
         result.addAll(subListA);
         result.addAll(subListB);
         return result;
      }
      else
      {
         List<T> result = new ArrayList<>(2);
         result.add(firstPoint);
         result.add(lastPoint);
         return result;
      }
   }
}
