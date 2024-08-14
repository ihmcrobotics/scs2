package net.javainthebox.caraibe.svg;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.StrokeLineCap;

public class SVGLoaderTest
{

   public SVGLoaderTest()
   {
   }

   @BeforeAll
   public static void setUpClass() throws Exception
   {
   }

   @AfterAll
   public static void tearDownClass() throws Exception
   {
   }

   @BeforeEach
   public void setUp()
   {
   }

   @AfterEach
   public void tearDown()
   {
   }

   @Test
   public void loadRectangle()
   {
      SVGContent content = SVGLoader.load("rectangle.svg");

      assertNotNull(content);

      Rectangle rect = (Rectangle) content.getNode("rect");
      assertEquals(100.0, rect.getX(), 0.0001);
      assertEquals(50.0, rect.getY(), 0.0001);
      assertEquals(100.0, rect.getWidth(), 0.0001);
      assertEquals(80.0, rect.getHeight(), 0.0001);
      assertEquals(Color.web("#FFFFFF").getRed(), ((Color) rect.getFill()).getRed(), 0.0001);
      assertEquals(Color.web("#FFFFFF").getGreen(), ((Color) rect.getFill()).getGreen(), 0.0001);
      assertEquals(Color.web("#FFFFFF").getBlue(), ((Color) rect.getFill()).getBlue(), 0.0001);
      assertEquals(Color.web("#FFFFFF").getOpacity(), ((Color) rect.getFill()).getOpacity(), 0.0001);
      assertEquals(Color.web("#000000").getRed(), ((Color) rect.getStroke()).getRed(), 0.0001);
      assertEquals(Color.web("#000000").getGreen(), ((Color) rect.getStroke()).getGreen(), 0.0001);
      assertEquals(Color.web("#000000").getBlue(), ((Color) rect.getStroke()).getBlue(), 0.0001);
      assertEquals(Color.web("#000000").getOpacity(), ((Color) rect.getStroke()).getOpacity(), 0.0001);
   }

   @Test
   public void loadGroup()
   {
      SVGContent content = SVGLoader.load("group.svg");

      assertNotNull(content);

      Group group = content.getGroup("group");
      assertNotNull(group);

      Node node = group.getChildren().get(0);
      assertTrue(node instanceof Circle);

      Circle circle = (Circle) node;
      assertEquals(100.0, circle.getCenterX(), 0.0001);
      assertEquals(100.0, circle.getCenterY(), 0.0001);
      assertEquals(50.0, circle.getRadius(), 0.0001);
      assertEquals(Color.web("#FF0000").getRed(), ((Color) circle.getFill()).getRed(), 0.0001);
      assertEquals(Color.web("#FF0000").getGreen(), ((Color) circle.getFill()).getGreen(), 0.0001);
      assertEquals(Color.web("#FF0000").getBlue(), ((Color) circle.getFill()).getBlue(), 0.0001);
      assertEquals(Color.web("#FF0000").getOpacity(), ((Color) circle.getFill()).getOpacity(), 0.0001);
      assertEquals(Color.web("#000000").getRed(), ((Color) circle.getStroke()).getRed(), 0.0001);
      assertEquals(Color.web("#000000").getGreen(), ((Color) circle.getStroke()).getGreen(), 0.0001);
      assertEquals(Color.web("#000000").getBlue(), ((Color) circle.getStroke()).getBlue(), 0.0001);
      assertEquals(Color.web("#000000").getOpacity(), ((Color) circle.getStroke()).getOpacity(), 0.0001);
   }

   @Test
   public void loadLines()
   {
      SVGContent content = SVGLoader.load("lines.svg");

      assertNotNull(content);

      Line line1 = (Line) content.getNode("line1");
      assertEquals(50, line1.getStartX(), 0.0001);
      assertEquals(60, line1.getStartY(), 0.0001);
      assertEquals(150, line1.getEndX(), 0.0001);
      assertEquals(60, line1.getEndY(), 0.0001);
      assertNull(line1.getFill());
      Color stroke = (Color) line1.getStroke();
      assertEquals(1.0, stroke.getRed(), 0.0001);

      Line line2 = (Line) content.getNode("line2");
      assertEquals(50, line2.getStartX(), 0.0001);
      assertEquals(80, line2.getStartY(), 0.0001);
      assertEquals(150, line2.getEndX(), 0.0001);
      assertEquals(80, line2.getEndY(), 0.0001);
      assertNull(line2.getFill());
      stroke = (Color) line2.getStroke();
      assertEquals(0.0, stroke.getRed(), 0.0001);
      assertEquals(2.0, line2.getStrokeWidth(), 0.0001);

      Line line3 = (Line) content.getNode("line3");
      assertEquals(50, line3.getStartX(), 0.0001);
      assertEquals(100, line3.getStartY(), 0.0001);
      assertEquals(150, line3.getEndX(), 0.0001);
      assertEquals(100, line3.getEndY(), 0.0001);
      assertNull(line3.getFill());
      stroke = (Color) line3.getStroke();
      assertEquals(0.0, stroke.getRed(), 0.0001);
      assertEquals(3.0, line3.getStrokeWidth(), 0.0001);
      assertEquals(StrokeLineCap.SQUARE, line3.getStrokeLineCap());

      Line line4 = (Line) content.getNode("line4");
      assertEquals(50, line4.getStartX(), 0.0001);
      assertEquals(120, line4.getStartY(), 0.0001);
      assertEquals(150, line4.getEndX(), 0.0001);
      assertEquals(120, line4.getEndY(), 0.0001);
      assertNull(line4.getFill());
      stroke = (Color) line4.getStroke();
      assertEquals(1.0, stroke.getRed(), 0.0001);
      assertEquals(4.0, line4.getStrokeWidth(), 0.0001);
      assertEquals(StrokeLineCap.ROUND, line4.getStrokeLineCap());

      Line line5 = (Line) content.getNode("line5");
      assertEquals(50, line5.getStartX(), 0.0001);
      assertEquals(140, line5.getStartY(), 0.0001);
      assertEquals(150, line5.getEndX(), 0.0001);
      assertEquals(140, line5.getEndY(), 0.0001);
      assertNull(line5.getFill());
      stroke = (Color) line5.getStroke();
      assertEquals(0.0, stroke.getRed(), 0.0001);
      ObservableList<Double> dasharray = line5.getStrokeDashArray();

      assertEquals(4.0, dasharray.get(0), 0.0001);
      assertEquals(8.0, dasharray.get(1), 0.0001);
   }
}
