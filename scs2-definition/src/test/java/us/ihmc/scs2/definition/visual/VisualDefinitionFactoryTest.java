package us.ihmc.scs2.definition.visual;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class VisualDefinitionFactoryTest
{
   static final double EPS = 1e-10;
   
   @Test
   public void testAddArrowPreviousTransform()
   {
      Random random = new Random(23943L);
      VisualDefinitionFactory v = new VisualDefinitionFactory();

      // Set a non-zero initial transform
      RigidBodyTransformReadOnly initialTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      v.appendTransform(initialTransform);
      v.saveCurrentTransform();

      RigidBodyTransformReadOnly t1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      v.appendTransform(t1);
      v.addArrow(EuclidCoreRandomTools.nextAxis3D(random), random.nextDouble());

      // reset to the previous saved transform, i.e. initialTransform in this case
      v.resetCurrentTransform();

      EuclidCoreTestTools.assertGeometricallyEquals(new AffineTransform(initialTransform), v.getCurrentTransform(), EPS);
   }
   
   @Test
   public void testAddShapePreviousTransform()
   {
      Random random = new Random(23943L);
      VisualDefinitionFactory v = new VisualDefinitionFactory();
      
      // Set a non-zero initial transform
      RigidBodyTransformReadOnly initialTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      v.appendTransform(initialTransform);
      v.saveCurrentTransform();
      
      RigidBodyTransformReadOnly t1 = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      v.appendTransform(t1);
      v.addShape(EuclidShapeRandomTools.nextShape3D(random));
      
      // reset to the previous saved transform, i.e. initialTransform in this case
      v.resetCurrentTransform();
      
      EuclidCoreTestTools.assertGeometricallyEquals(new AffineTransform(initialTransform), v.getCurrentTransform(), EPS);
   }
}

