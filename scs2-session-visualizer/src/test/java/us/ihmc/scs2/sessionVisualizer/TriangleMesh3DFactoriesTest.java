package us.ihmc.scs2.sessionVisualizer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;
import static us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories.Box;
import static us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories.Sphere;
import static us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories.Torus;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;

public class TriangleMesh3DFactoriesTest
{

   @Test
   public void testCombine()
   {
      { // Combine 1 Torus and 1 Sphere, with index adjustment
         TriangleMesh3DDefinition originalTorus = Torus(0.15, 0.05, 64);
         TriangleMesh3DDefinition originalSphere = Sphere(0.1, 32, 32);

         TriangleMesh3DDefinition torus = Torus(0.15, 0.05, 64);
         TriangleMesh3DDefinition sphere = Sphere(0.1, 32, 32);
         TriangleMesh3DDefinition combined = TriangleMesh3DFactories.combine(false, false, torus, sphere);

         assertEquals(originalTorus.getVertices().length + originalSphere.getVertices().length, combined.getVertices().length);
         assertEquals(originalTorus.getTextures().length + originalSphere.getTextures().length, combined.getTextures().length);
         assertEquals(originalTorus.getNormals().length + originalSphere.getNormals().length, combined.getNormals().length);
         assertEquals(originalTorus.getTriangleIndices().length + originalSphere.getTriangleIndices().length, combined.getTriangleIndices().length);

         assertIterableEquals(Arrays.asList(originalTorus.getVertices()), Arrays.asList(combined.getVertices()).subList(0, originalTorus.getVertices().length));
         assertIterableEquals(Arrays.asList(originalTorus.getTextures()), Arrays.asList(combined.getTextures()).subList(0, originalTorus.getTextures().length));
         assertIterableEquals(Arrays.asList(originalTorus.getNormals()), Arrays.asList(combined.getNormals()).subList(0, originalTorus.getNormals().length));
         assertIterableEquals(intArrayToList(originalTorus.getTriangleIndices()),
                              intArrayToList(combined.getTriangleIndices()).subList(0, originalTorus.getTriangleIndices().length));

         assertIterableEquals(Arrays.asList(originalSphere.getVertices()),
                              Arrays.asList(combined.getVertices()).subList(originalTorus.getVertices().length, combined.getVertices().length));
         assertIterableEquals(Arrays.asList(originalSphere.getTextures()),
                              Arrays.asList(combined.getTextures()).subList(originalTorus.getTextures().length, combined.getTextures().length));
         assertIterableEquals(Arrays.asList(originalSphere.getNormals()),
                              Arrays.asList(combined.getNormals()).subList(originalTorus.getNormals().length, combined.getNormals().length));
         assertIterableEquals(intArrayToList(originalSphere.getTriangleIndices()),
                              intArrayToList(combined.getTriangleIndices()).subList(originalTorus.getTriangleIndices().length,
                                                                                    combined.getTriangleIndices().length));
      }

      { // Combine 1 Torus and 1 Sphere, with index adjustment
         TriangleMesh3DDefinition originalShape1 = Torus(0.15, 0.05, 64);
         TriangleMesh3DDefinition originalShape2 = Sphere(0.1, 32, 32);

         TriangleMesh3DDefinition shape1 = Torus(0.15, 0.05, 64);
         TriangleMesh3DDefinition shape2 = Sphere(0.1, 32, 32);
         TriangleMesh3DDefinition combined = TriangleMesh3DFactories.combine(true, false, shape1, shape2);

         Point3D32[] v1 = originalShape1.getVertices();
         Point2D32[] t1 = originalShape1.getTextures();
         Vector3D32[] n1 = originalShape1.getNormals();
         int[] i1 = originalShape1.getTriangleIndices();

         Point3D32[] v2 = originalShape2.getVertices();
         Point2D32[] t2 = originalShape2.getTextures();
         Vector3D32[] n2 = originalShape2.getNormals();
         int[] i2 = originalShape2.getTriangleIndices();

         Point3D32[] vc = combined.getVertices();
         Point2D32[] tc = combined.getTextures();
         Vector3D32[] nc = combined.getNormals();
         int[] ic = combined.getTriangleIndices();

         assertEquals(v1.length + v2.length, vc.length);
         assertEquals(t1.length + t2.length, tc.length);
         assertEquals(n1.length + n2.length, nc.length);
         assertEquals(i1.length + i2.length, ic.length);

         assertIterableEquals(Arrays.asList(v1), Arrays.asList(vc).subList(0, v1.length));
         assertIterableEquals(Arrays.asList(t1), Arrays.asList(tc).subList(0, t1.length));
         assertIterableEquals(Arrays.asList(n1), Arrays.asList(nc).subList(0, n1.length));
         assertIterableEquals(intArrayToList(i1), intArrayToList(ic).subList(0, i1.length));

         assertIterableEquals(Arrays.asList(v2), Arrays.asList(vc).subList(v1.length, vc.length));
         assertIterableEquals(Arrays.asList(t2), Arrays.asList(tc).subList(t1.length, tc.length));
         assertIterableEquals(Arrays.asList(n2), Arrays.asList(nc).subList(n1.length, nc.length));
         assertIterableEquals(intArrayToList(shift(i2, v1.length)), intArrayToList(ic).subList(i1.length, ic.length));
      }

      { // Test with 3 shapes
         TriangleMesh3DDefinition originalShape1 = Torus(0.15, 0.05, 16);
         TriangleMesh3DDefinition originalShape2 = Sphere(0.1, 8, 8);
         TriangleMesh3DDefinition originalShape3 = Box(0.1, 0.2, 0.3, true);

         TriangleMesh3DDefinition shape1 = Torus(0.15, 0.05, 16);
         TriangleMesh3DDefinition shape2 = Sphere(0.1, 8, 8);
         TriangleMesh3DDefinition shape3 = Box(0.1, 0.2, 0.3, true);
         shape1.setName("shape1");
         shape2.setName("shape2");
         shape3.setName("shape3");
         TriangleMesh3DDefinition combined = TriangleMesh3DFactories.combine(true, false, shape1, shape2, shape3);

         Point3D32[] v1 = originalShape1.getVertices();
         Point2D32[] t1 = originalShape1.getTextures();
         Vector3D32[] n1 = originalShape1.getNormals();
         int[] i1 = originalShape1.getTriangleIndices();

         Point3D32[] v2 = originalShape2.getVertices();
         Point2D32[] t2 = originalShape2.getTextures();
         Vector3D32[] n2 = originalShape2.getNormals();
         int[] i2 = originalShape2.getTriangleIndices();

         Point3D32[] v3 = originalShape3.getVertices();
         Point2D32[] t3 = originalShape3.getTextures();
         Vector3D32[] n3 = originalShape3.getNormals();
         int[] i3 = originalShape3.getTriangleIndices();

         Point3D32[] vc = combined.getVertices();
         Point2D32[] tc = combined.getTextures();
         Vector3D32[] nc = combined.getNormals();
         int[] ic = combined.getTriangleIndices();

         assertEquals(v1.length + v2.length + v3.length, vc.length);
         assertEquals(t1.length + t2.length + t3.length, tc.length);
         assertEquals(n1.length + n2.length + n3.length, nc.length);
         assertEquals(i1.length + i2.length + i3.length, ic.length);

         assertIterableEquals(Arrays.asList(v1), Arrays.asList(vc).subList(0, v1.length));
         assertIterableEquals(Arrays.asList(t1), Arrays.asList(tc).subList(0, t1.length));
         assertIterableEquals(Arrays.asList(n1), Arrays.asList(nc).subList(0, n1.length));
         assertIterableEquals(intArrayToList(i1), intArrayToList(ic).subList(0, i1.length));

         assertIterableEquals(Arrays.asList(v2), Arrays.asList(vc).subList(v1.length, v1.length + v2.length));
         assertIterableEquals(Arrays.asList(t2), Arrays.asList(tc).subList(t1.length, t1.length + t2.length));
         assertIterableEquals(Arrays.asList(n2), Arrays.asList(nc).subList(n1.length, n1.length + n2.length));
         assertIterableEquals(intArrayToList(shift(i2, v1.length)), intArrayToList(ic).subList(i1.length, i1.length + i2.length));

         assertIterableEquals(Arrays.asList(v3), Arrays.asList(vc).subList(v1.length + v2.length, vc.length));
         assertIterableEquals(Arrays.asList(t3), Arrays.asList(tc).subList(t1.length + t2.length, tc.length));
         assertIterableEquals(Arrays.asList(n3), Arrays.asList(nc).subList(n1.length + n2.length, nc.length));
         assertIterableEquals(intArrayToList(shift(i3, v1.length + v2.length)), intArrayToList(ic).subList(i1.length + i2.length, ic.length));

      }
   }

   private static List<Integer> intArrayToList(int[] array)
   {
      List<Integer> list = new ArrayList<>(array.length);
      for (int e : array)
      {
         list.add(e);
      }
      return list;
   }

   private static int[] shift(int[] array, int shift)
   {
      int[] out = new int[array.length];
      for (int i = 0; i < array.length; i++)
      {
         out[i] = array[i] + shift;
      }
      return out;
   }
}
