package us.ihmc.scs2.definition.yoGraphic;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;

public class YoGraphicDefinitionFactoryTest
{
   @Test
   public void testDefaultPoint2DGraphic()
   {
      assertEquals(DefaultPoint2DGraphic.PLUS.getGraphicName(), "Plus");
      assertEquals(DefaultPoint2DGraphic.CROSS.getGraphicName(), "Cross");
      assertEquals(DefaultPoint2DGraphic.CIRCLE_PLUS.getGraphicName(), "Circle plus");
      assertEquals(DefaultPoint2DGraphic.CIRCLE_CROSS.getGraphicName(), "Circle cross");
      assertEquals(DefaultPoint2DGraphic.DIAMOND.getGraphicName(), "Diamond");
      assertEquals(DefaultPoint2DGraphic.DIAMOND_PLUS.getGraphicName(), "Diamond plus");
      assertEquals(DefaultPoint2DGraphic.SQUARE.getGraphicName(), "Square");
      assertEquals(DefaultPoint2DGraphic.SQUARE_CROSS.getGraphicName(), "Square cross");
   }

}
