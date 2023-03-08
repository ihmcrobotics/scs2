package us.ihmc.scs2.definition;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

import org.apache.commons.lang3.RandomStringUtils;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBADoubleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBAIntDefinition;
import us.ihmc.scs2.definition.yoComposite.YoColorRGBASingleDefinition;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoYawPitchRollDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphic3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicBox3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCapsule3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCone3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCylinder3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicEllipsoid3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicLine2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPointcloud2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPointcloud3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygon2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygonExtruded3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolynomial3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicRamp3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoListDefinition;

public class DefinitionRandomTools
{
   public static String nextName(Random random)
   {
      return nextName(random, false);
   }

   public static String nextName(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      int count = random.nextInt(20) + 2;
      return RandomStringUtils.random(count, 0, 0, true, true, null, random);
   }

   public static String nextDoubleFieldValue(Random random)
   {
      return nextDoubleFieldValue(random, false);
   }

   public static String nextDoubleFieldValue(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      return random.nextBoolean() ? nextName(random) : Double.toString(EuclidCoreRandomTools.nextDouble(random, 10.0));
   }

   public static String nextIntFieldValue(Random random)
   {
      return nextIntFieldValue(random, false);
   }

   public static String nextIntFieldValue(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      return random.nextBoolean() ? nextName(random) : Integer.toString(random.nextInt(1000) - 500);
   }

   public static String nextBooleanFieldValue(Random random)
   {
      return nextBooleanFieldValue(random, false);
   }

   public static String nextBooleanFieldValue(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      return random.nextBoolean() ? nextName(random) : Boolean.toString(random.nextBoolean());
   }

   public static YoTuple2DDefinition nextYoTuple2DDefinition(Random random)
   {
      return nextYoTuple2DDefinition(random, false);
   }

   public static YoTuple2DDefinition nextYoTuple2DDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      String x = nextDoubleFieldValue(random);
      String y = nextDoubleFieldValue(random);
      String referenceFrame = nextDoubleFieldValue(random, true);
      return new YoTuple2DDefinition(x, y, referenceFrame);
   }

   public static YoTuple3DDefinition nextYoTuple3DDefinition(Random random)
   {
      return nextYoTuple3DDefinition(random, false);
   }

   public static YoTuple3DDefinition nextYoTuple3DDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      String x = nextDoubleFieldValue(random);
      String y = nextDoubleFieldValue(random);
      String z = nextDoubleFieldValue(random);
      String referenceFrame = nextDoubleFieldValue(random, true);
      return new YoTuple3DDefinition(x, y, z, referenceFrame);
   }

   public static YoOrientation3DDefinition nextYoOrientation3DDefinition(Random random)
   {
      return nextYoOrientation3DDefinition(random, false);
   }

   public static YoOrientation3DDefinition nextYoOrientation3DDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;

      switch (random.nextInt(2))
      {
         case 0:
            return nextYoQuaternionDefinition(random);
         default:
            return nextYoYawPitchRollDefinition(random);
      }
   }

   public static YoQuaternionDefinition nextYoQuaternionDefinition(Random random)
   {
      return nextYoQuaternionDefinition(random, false);
   }

   public static YoQuaternionDefinition nextYoQuaternionDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      String x = nextDoubleFieldValue(random);
      String y = nextDoubleFieldValue(random);
      String z = nextDoubleFieldValue(random);
      String s = nextDoubleFieldValue(random);
      String referenceFrame = nextDoubleFieldValue(random, true);
      return new YoQuaternionDefinition(x, y, z, s, referenceFrame);
   }

   public static YoYawPitchRollDefinition nextYoYawPitchRollDefinition(Random random)
   {
      return nextYoYawPitchRollDefinition(random, false);
   }

   public static YoYawPitchRollDefinition nextYoYawPitchRollDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      String yaw = nextDoubleFieldValue(random);
      String pitch = nextDoubleFieldValue(random);
      String roll = nextDoubleFieldValue(random);
      String referenceFrame = nextDoubleFieldValue(random, true);
      return new YoYawPitchRollDefinition(yaw, pitch, roll, referenceFrame);
   }

   public static PaintDefinition nextPaintDefinition(Random random)
   {
      return nextPaintDefinition(random, false);
   }

   public static PaintDefinition nextPaintDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      switch (random.nextInt(4))
      {
         case 0:
            return nextColorDefinition(random);
         case 1:
            return nextYoColorRGBADoubleDefinition(random);
         case 2:
            return nextYoColorRGBAIntDefinition(random);
         default:
            return nextYoColorRGBASingleDefinition(random);
      }
   }

   public static ColorDefinition nextColorDefinition(Random random)
   {
      return nextColorDefinition(random, false);
   }

   public static ColorDefinition nextColorDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      return ColorDefinition.rgba(random.nextInt());
   }

   public static YoColorRGBADoubleDefinition nextYoColorRGBADoubleDefinition(Random random)
   {
      return nextYoColorRGBADoubleDefinition(random, false);
   }

   public static YoColorRGBADoubleDefinition nextYoColorRGBADoubleDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      String red = nextDoubleFieldValue(random);
      String green = nextDoubleFieldValue(random);
      String blue = nextDoubleFieldValue(random);
      String alpha = nextDoubleFieldValue(random, true);
      return new YoColorRGBADoubleDefinition(red, green, blue, alpha);
   }

   public static YoColorRGBAIntDefinition nextYoColorRGBAIntDefinition(Random random)
   {
      return nextYoColorRGBAIntDefinition(random, false);
   }

   public static YoColorRGBAIntDefinition nextYoColorRGBAIntDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      String red = nextIntFieldValue(random);
      String green = nextIntFieldValue(random);
      String blue = nextIntFieldValue(random);
      String alpha = nextIntFieldValue(random, true);
      return new YoColorRGBAIntDefinition(red, green, blue, alpha);
   }

   public static YoColorRGBASingleDefinition nextYoColorRGBASingleDefinition(Random random)
   {
      return nextYoColorRGBASingleDefinition(random, false);
   }

   public static YoColorRGBASingleDefinition nextYoColorRGBASingleDefinition(Random random, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      String rgba = nextIntFieldValue(random);
      return new YoColorRGBASingleDefinition(rgba);
   }

   public static YoListDefinition nextYoListDefinition(Random random)
   {
      YoListDefinition next = new YoListDefinition();
      switch (random.nextInt(8))
      {
         case 0:
            // Leave the list null
            break;
         case 1:
            // Make an empty list
            next.setElements(new ArrayList<>());
            break;
         default:
            int size = random.nextInt(20) + 1;
            for (int i = 0; i < size; i++)
            {
               next.addElement(nextDoubleFieldValue(random));
            }
      }
      next.setSize(nextIntFieldValue(random, true));
      return next;
   }

   public static <T> List<T> nextListOf(Random random, Function<Random, T> randomGenerator, int size)
   {
      return nextListOf(random, randomGenerator, size, false);
   }

   public static <T> List<T> nextListOf(Random random, Function<Random, T> randomGenerator, int size, boolean nillable)
   {
      if (nillable && random.nextBoolean())
         return null;
      List<T> next = new ArrayList<>(size);
      for (int i = 0; i < size; i++)
      {
         next.add(randomGenerator.apply(random));
      }
      return next;
   }

   public static YoGraphicDefinition nextYoGraphicDefinition(Random random)
   {
      switch (random.nextInt(3))
      {
         case 0:
            return nextYoGraphic2DDefinition(random);
         case 1:
            return nextYoGraphic3DDefinition(random);
         default:
            return nextYoGraphicGroupDefinition(random);
      }
   }

   public static YoGraphicGroupDefinition nextYoGraphicGroupDefinition(Random random)
   {
      return nextYoGraphicGroupDefinition(random, 10);
   }

   public static YoGraphicGroupDefinition nextYoGraphicGroupDefinition(Random random, int maxDepth)
   {
      int numberOfGraphic2D = random.nextInt(10);
      int numberOfGraphic3D = random.nextInt(10);
      int numberOfSubGroup = random.nextInt(5);

      YoGraphicGroupDefinition next = new YoGraphicGroupDefinition();
      randomizeYoGraphicDefinitionProperties(random, next);

      for (int i = 0; i < numberOfGraphic2D; i++)
         next.addChild(nextYoGraphic2DDefinition(random));
      for (int i = 0; i < numberOfGraphic3D; i++)
         next.addChild(nextYoGraphic3DDefinition(random));

      if (maxDepth <= 1)
         return next;

      for (int i = 0; i < numberOfSubGroup; i++)
         next.addChild(nextYoGraphicGroupDefinition(random, maxDepth - 1));
      return next;
   }

   public static YoGraphic2DDefinition nextYoGraphic2DDefinition(Random random)
   {
      switch (random.nextInt(4))
      {
         case 0:
            return nextYoGraphicLine2DDefinition(random);
         case 1:
            return nextYoGraphicPoint2DDefinition(random);
         case 2:
            return nextYoGraphicPointcloud2DDefinition(random);
         default:
            return nextYoGraphicPolygon2DDefinition(random);
      }
   }

   public static YoGraphicLine2DDefinition nextYoGraphicLine2DDefinition(Random random)
   {
      YoGraphicLine2DDefinition next = new YoGraphicLine2DDefinition();
      randomizeYoGraphic2DDefinitionProperties(random, next);
      next.setOrigin(nextYoTuple2DDefinition(random, true));
      next.setDirection(nextYoTuple2DDefinition(random, true));
      next.setDestination(nextYoTuple2DDefinition(random, true));
      return next;
   }

   public static YoGraphicPoint2DDefinition nextYoGraphicPoint2DDefinition(Random random)
   {
      YoGraphicPoint2DDefinition next = new YoGraphicPoint2DDefinition();
      randomizeYoGraphic2DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple2DDefinition(random, true));
      next.setSize(nextDoubleFieldValue(random, true));
      next.setGraphicName(nextName(random, true));
      return next;
   }

   public static YoGraphicPointcloud2DDefinition nextYoGraphicPointcloud2DDefinition(Random random)
   {
      YoGraphicPointcloud2DDefinition next = new YoGraphicPointcloud2DDefinition();
      randomizeYoGraphic2DDefinitionProperties(random, next);
      next.setPoints(nextListOf(random, r -> nextYoTuple2DDefinition(r), random.nextInt(20)));
      next.setNumberOfPoints(nextIntFieldValue(random, true));
      next.setSize(nextDoubleFieldValue(random, true));
      next.setGraphicName(nextName(random, true));
      return next;
   }

   public static YoGraphicPolygon2DDefinition nextYoGraphicPolygon2DDefinition(Random random)
   {
      YoGraphicPolygon2DDefinition next = new YoGraphicPolygon2DDefinition();
      randomizeYoGraphic2DDefinitionProperties(random, next);
      next.setVertices(nextListOf(random, r -> nextYoTuple2DDefinition(r), random.nextInt(20)));
      next.setNumberOfVertices(nextIntFieldValue(random, true));
      return next;
   }

   public static YoGraphic3DDefinition nextYoGraphic3DDefinition(Random random)
   {
      switch (random.nextInt(13))
      {
         case 0:
            return nextYoGraphicArrow3DDefinition(random);
         case 1:
            return nextYoGraphicBox3DDefinition(random);
         case 2:
            return nextYoGraphicCapsule3DDefinition(random);
         case 3:
            return nextYoGraphicCone3DDefinition(random);
         case 4:
            return nextYoGraphicConvexPolytope3DDefinition(random);
         case 5:
            return nextYoGraphicCoordinateSystem3DDefinition(random);
         case 6:
            return nextYoGraphicCylinder3DDefinition(random);
         case 7:
            return nextYoGraphicEllipsoid3DDefinition(random);
         case 8:
            return nextYoGraphicPoint3DDefinition(random);
         case 9:
            return nextYoGraphicPointcloud3DDefinition(random);
         case 10:
            return nextYoGraphicPolygonExtruded3DDefinition(random);
         case 11:
            return nextYoGraphicPolynomial3DDefinition(random);
         default:
            return nextYoGraphicRamp3DDefinition(random);
      }
   }

   public static YoGraphicArrow3DDefinition nextYoGraphicArrow3DDefinition(Random random)
   {
      YoGraphicArrow3DDefinition next = new YoGraphicArrow3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setOrigin(nextYoTuple3DDefinition(random, true));
      next.setDirection(nextYoTuple3DDefinition(random, true));
      next.setScaleLength(random.nextBoolean());
      next.setBodyLength(nextDoubleFieldValue(random));
      next.setHeadLength(nextDoubleFieldValue(random));
      next.setScaleRadius(random.nextBoolean());
      next.setBodyRadius(nextDoubleFieldValue(random));
      next.setHeadRadius(nextDoubleFieldValue(random));
      return next;
   }

   public static YoGraphicBox3DDefinition nextYoGraphicBox3DDefinition(Random random)
   {
      YoGraphicBox3DDefinition next = new YoGraphicBox3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setOrientation(nextYoOrientation3DDefinition(random, true));
      next.setSize(nextYoTuple3DDefinition(random, true));
      return next;
   }

   public static YoGraphicCapsule3DDefinition nextYoGraphicCapsule3DDefinition(Random random)
   {
      YoGraphicCapsule3DDefinition next = new YoGraphicCapsule3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setCenter(nextYoTuple3DDefinition(random, true));
      next.setAxis(nextYoTuple3DDefinition(random, true));
      next.setLength(nextDoubleFieldValue(random));
      next.setRadius(nextDoubleFieldValue(random));
      return next;
   }

   public static YoGraphicCone3DDefinition nextYoGraphicCone3DDefinition(Random random)
   {
      YoGraphicCone3DDefinition next = new YoGraphicCone3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setAxis(nextYoTuple3DDefinition(random, true));
      next.setHeight(nextDoubleFieldValue(random));
      next.setRadius(nextDoubleFieldValue(random));
      return next;
   }

   public static YoGraphicConvexPolytope3DDefinition nextYoGraphicConvexPolytope3DDefinition(Random random)
   {
      YoGraphicConvexPolytope3DDefinition next = new YoGraphicConvexPolytope3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setOrientation(nextYoOrientation3DDefinition(random, true));
      next.setVertices(nextListOf(random, r -> nextYoTuple3DDefinition(r), random.nextInt(20)));
      next.setNumberOfVertices(nextIntFieldValue(random, true));
      return next;
   }

   public static YoGraphicCoordinateSystem3DDefinition nextYoGraphicCoordinateSystem3DDefinition(Random random)
   {
      YoGraphicCoordinateSystem3DDefinition next = new YoGraphicCoordinateSystem3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setOrientation(nextYoOrientation3DDefinition(random, true));
      next.setBodyLength(nextDoubleFieldValue(random));
      next.setHeadLength(nextDoubleFieldValue(random));
      next.setBodyRadius(nextDoubleFieldValue(random));
      next.setHeadRadius(nextDoubleFieldValue(random));
      return next;
   }

   public static YoGraphicCylinder3DDefinition nextYoGraphicCylinder3DDefinition(Random random)
   {
      YoGraphicCylinder3DDefinition next = new YoGraphicCylinder3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setCenter(nextYoTuple3DDefinition(random, true));
      next.setAxis(nextYoTuple3DDefinition(random, true));
      next.setLength(nextDoubleFieldValue(random));
      next.setRadius(nextDoubleFieldValue(random));
      return next;
   }

   public static YoGraphicEllipsoid3DDefinition nextYoGraphicEllipsoid3DDefinition(Random random)
   {
      YoGraphicEllipsoid3DDefinition next = new YoGraphicEllipsoid3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setOrientation(nextYoOrientation3DDefinition(random, true));
      next.setRadii(nextYoTuple3DDefinition(random, true));
      return next;
   }

   public static YoGraphicPoint3DDefinition nextYoGraphicPoint3DDefinition(Random random)
   {
      YoGraphicPoint3DDefinition next = new YoGraphicPoint3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setSize(nextDoubleFieldValue(random, true));
      next.setGraphicName(nextName(random, true));
      return next;
   }

   public static YoGraphicPointcloud3DDefinition nextYoGraphicPointcloud3DDefinition(Random random)
   {
      YoGraphicPointcloud3DDefinition next = new YoGraphicPointcloud3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPoints(nextListOf(random, r -> nextYoTuple3DDefinition(r), random.nextInt(20)));
      next.setNumberOfPoints(nextIntFieldValue(random, true));
      next.setSize(nextDoubleFieldValue(random, true));
      next.setGraphicName(nextName(random, true));
      return next;
   }

   public static YoGraphicPolygonExtruded3DDefinition nextYoGraphicPolygonExtruded3DDefinition(Random random)
   {
      YoGraphicPolygonExtruded3DDefinition next = new YoGraphicPolygonExtruded3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setOrientation(nextYoOrientation3DDefinition(random, true));
      next.setVertices(nextListOf(random, r -> nextYoTuple2DDefinition(r), random.nextInt(20)));
      next.setNumberOfVertices(nextIntFieldValue(random, true));
      next.setThickness(nextDoubleFieldValue(random));
      return next;
   }

   public static YoGraphicPolynomial3DDefinition nextYoGraphicPolynomial3DDefinition(Random random)
   {
      YoGraphicPolynomial3DDefinition next = new YoGraphicPolynomial3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setCoefficientsX(nextYoListDefinition(random));
      next.setCoefficientsY(nextYoListDefinition(random));
      next.setCoefficientsZ(nextYoListDefinition(random));
      next.setReferenceFrame(nextName(random, true));
      next.setStartTime(nextDoubleFieldValue(random));
      next.setEndTime(nextDoubleFieldValue(random));
      next.setSize(nextDoubleFieldValue(random));
      next.setTimeResolution(nextIntFieldValue(random));
      next.setNumberOfDivisions(nextIntFieldValue(random));
      return next;
   }

   public static YoGraphicRamp3DDefinition nextYoGraphicRamp3DDefinition(Random random)
   {
      YoGraphicRamp3DDefinition next = new YoGraphicRamp3DDefinition();
      randomizeYoGraphic3DDefinitionProperties(random, next);
      next.setPosition(nextYoTuple3DDefinition(random, true));
      next.setOrientation(nextYoOrientation3DDefinition(random, true));
      next.setSize(nextYoTuple3DDefinition(random, true));
      return next;
   }

   public static void randomizeYoGraphicDefinitionProperties(Random random, YoGraphicDefinition definition)
   {
      definition.setName(nextName(random));
      definition.setVisible(random.nextBoolean());
   }

   public static void randomizeYoGraphic2DDefinitionProperties(Random random, YoGraphic2DDefinition definition)
   {
      randomizeYoGraphicDefinitionProperties(random, definition);
      definition.setFillColor(random.nextBoolean() ? null : nextPaintDefinition(random));
      definition.setStrokeColor(random.nextBoolean() ? null : nextPaintDefinition(random));
      definition.setStrokeWidth(nextDoubleFieldValue(random));
   }

   public static void randomizeYoGraphic3DDefinitionProperties(Random random, YoGraphic3DDefinition definition)
   {
      randomizeYoGraphicDefinitionProperties(random, definition);
      definition.setColor(random.nextBoolean() ? null : nextPaintDefinition(random));
   }
}
