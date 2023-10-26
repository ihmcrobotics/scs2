package us.ihmc.scs2.simulation.shapes;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.geometry.shapes.STPShape3DTools;

public class STPShape3DToolsTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testArePoint3DsSameSideOfPlane3D()
   {
      Random random = new Random(435234657);

      for (int i = 0; i < ITERATIONS; i++)
      { // Create the queries such that they're on the same side of the plane.
         Point3D firstPointOnPlane = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D firstPlaneTangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Vector3D secondPlaneTangent = new Vector3D();
         secondPlaneTangent.cross(planeNormal, firstPlaneTangent);

         Point3D secondPointOnPlane = EuclidGeometryTools.orthogonalProjectionOnPlane3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0),
                                                                                        firstPointOnPlane,
                                                                                        planeNormal);

         // Used to determine the side of the plane
         double sign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D firstQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the side of the plane
         firstQuery.scaleAdd(sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, firstQuery);
         // Shifting the query without changing side w.r.t. the plane
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, firstQuery);
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, firstQuery);

         Point3D secondQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the side of the plane
         secondQuery.scaleAdd(sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, secondQuery);
         // Shifting the query without changing side w.r.t. the plane
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, secondQuery);
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, secondQuery);

         assertTrue(STPShape3DTools.arePoint3DsSameSideOfPlane3D(firstQuery, secondQuery, firstPointOnPlane, secondPointOnPlane, firstPlaneTangent));
      }
      for (int i = 0; i < ITERATIONS; i++)
      { // Create the queries such that they're each side of the plane.
         Point3D firstPointOnPlane = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
         Vector3D firstPlaneTangent = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Vector3D secondPlaneTangent = new Vector3D();
         secondPlaneTangent.cross(planeNormal, firstPlaneTangent);

         Point3D secondPointOnPlane = EuclidGeometryTools.orthogonalProjectionOnPlane3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0),
                                                                                        firstPointOnPlane,
                                                                                        planeNormal);

         // Used to determine the side of the plane
         double sign = random.nextBoolean() ? -1.0 : 1.0;

         Point3D firstQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the side of the plane
         firstQuery.scaleAdd(sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, firstQuery);
         // Shifting the query without changing side w.r.t. the plane
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, firstQuery);
         firstQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, firstQuery);

         Point3D secondQuery = new Point3D(firstPointOnPlane);
         // Shifting the query to the other side of the plane
         secondQuery.scaleAdd(-sign * EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), planeNormal, secondQuery);
         // Shifting the query without changing side w.r.t. the plane
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstPlaneTangent, secondQuery);
         secondQuery.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondPlaneTangent, secondQuery);

         assertFalse(STPShape3DTools.arePoint3DsSameSideOfPlane3D(firstQuery, secondQuery, firstPointOnPlane, secondPointOnPlane, firstPlaneTangent));
      }
   }
}
