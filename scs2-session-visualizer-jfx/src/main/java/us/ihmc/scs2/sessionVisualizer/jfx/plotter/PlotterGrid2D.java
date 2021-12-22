package us.ihmc.scs2.sessionVisualizer.jfx.plotter;

import java.awt.Toolkit;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleDoubleProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Node;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.transform.NonInvertibleTransformException;
import javafx.scene.transform.Transform;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXToEuclidConversions;

public class PlotterGrid2D
{
   private static final int SIGNIFICANT_FIGURES = 4;

   private final Canvas canvas = new Canvas(300, 500);
   private final ReadOnlyObjectProperty<Transform> localToSceneTransformProperty;

   private final DoubleProperty lineWidthProperty = new SimpleDoubleProperty(this, "lineWidthProperty", 0.125);
   private final ObjectProperty<Color> colorProperty = new SimpleObjectProperty<>(this, "colorProperty", Color.BLACK);
   private final ObjectProperty<Font> fontProperty = new SimpleObjectProperty<>(this, "fontProperty", Font.font("Verdana", 10.0));

   public PlotterGrid2D(ReadOnlyObjectProperty<Transform> localToSceneTransformProperty)
   {
      this.localToSceneTransformProperty = localToSceneTransformProperty;
   }

   public void update(double top, double left, double width, double height)
   {
      Point2D topLeftScene = new Point2D(left, top);
      Point2D bottomRightScene = new Point2D(left + width, top + height);

      Point2D topLeftLocal = sceneToLocal(topLeftScene);
      Point2D bottomRightLocal = sceneToLocal(bottomRightScene);

      Point2D startLocal = new Point2D();
      Point2D endLocal = new Point2D();

      if (topLeftLocal.getX() < bottomRightLocal.getX())
      {
         startLocal.setX(topLeftLocal.getX());
         endLocal.setX(bottomRightLocal.getX());
      }
      else
      {
         startLocal.setX(bottomRightLocal.getX());
         endLocal.setX(topLeftLocal.getX());
      }

      if (topLeftLocal.getY() < bottomRightLocal.getY())
      {
         startLocal.setY(topLeftLocal.getY());
         endLocal.setY(bottomRightLocal.getY());
      }
      else
      {
         startLocal.setY(bottomRightLocal.getY());
         endLocal.setY(topLeftLocal.getY());
      }

      Vector2D rangeLocal = new Vector2D();
      rangeLocal.sub(endLocal, startLocal);

      Vector2D gridSizeLocal = new Vector2D(calculateGridSize(width / rangeLocal.getX()), calculateGridSize(height / rangeLocal.getY()));

      Point2D gridStartLocal = new Point2D();
      gridStartLocal.setX(Math.ceil(startLocal.getX() / gridSizeLocal.getX()) * gridSizeLocal.getX());
      gridStartLocal.setY(Math.ceil(startLocal.getY() / gridSizeLocal.getY()) * gridSizeLocal.getY());

      int gridCountX = (int) Math.ceil(rangeLocal.getX() / gridSizeLocal.getX());
      int gridCountY = (int) Math.ceil(rangeLocal.getY() / gridSizeLocal.getY());

      Vector2D gridSizeScene = localToScene(gridSizeLocal);
      Point2D gridStartScene = localToScene(gridStartLocal);

      canvas.setWidth(width);
      canvas.setHeight(height);
      GraphicsContext gc = canvas.getGraphicsContext2D();
      gc.clearRect(0, 0, width, height);
      gc.setStroke(colorProperty.get());
      gc.setLineWidth(lineWidthProperty.get());

      for (int i = 0; i < gridCountX; i++)
      {
         double gridX = gridStartScene.getX() + i * gridSizeScene.getX();
         gc.strokeLine(gridX, top, gridX, top + height);
      }

      for (int i = 0; i < gridCountY; i++)
      {
         double gridY = gridStartScene.getY() + i * gridSizeScene.getY();
         gc.strokeLine(left, gridY, left + width, gridY);
      }

      // Now adding coordinate labels
      gc.setFont(fontProperty.get());
      double fontSize = fontProperty.get().getSize();

      for (int i = 0; i < gridCountX; i++)
      {
         double gridX = gridStartScene.getX() + i * gridSizeScene.getX();
         double localX = gridStartLocal.getX() + i * gridSizeLocal.getX();
         gc.fillText(FormattingTools.getFormattedToSignificantFigures(localX, SIGNIFICANT_FIGURES), gridX, top + fontSize);
      }

      for (int i = 0; i < gridCountY; i++)
      {
         double gridY = gridStartScene.getY() + i * gridSizeScene.getY();
         double localY = gridStartLocal.getY() + i * gridSizeLocal.getY();
         gc.fillText(FormattingTools.getFormattedToSignificantFigures(localY, SIGNIFICANT_FIGURES), left, gridY - 2.0);
      }
   }

   public Point2D sceneToLocal(Point2DReadOnly pointInScene)
   {
      Transform transform = localToSceneTransformProperty.get();
      try
      {
         return JavaFXToEuclidConversions.convertPoint2D(transform.inverseTransform(pointInScene.getX(), pointInScene.getY()));
      }
      catch (NonInvertibleTransformException e)
      {
         return null;
      }
   }

   public Point2D localToScene(Point2DReadOnly pointInLocal)
   {
      Transform transform = localToSceneTransformProperty.get();
      return JavaFXToEuclidConversions.convertPoint2D(transform.transform(pointInLocal.getX(), pointInLocal.getY()));
   }

   public Vector2D sceneToLocal(Vector2DReadOnly pointInScene)
   {
      Transform transform = localToSceneTransformProperty.get();
      try
      {
         return JavaFXToEuclidConversions.convertVector2D(transform.inverseDeltaTransform(pointInScene.getX(), pointInScene.getY()));
      }
      catch (NonInvertibleTransformException e)
      {
         return null;
      }
   }

   public Vector2D localToScene(Vector2DReadOnly pointInLocal)
   {
      Transform transform = localToSceneTransformProperty.get();
      return JavaFXToEuclidConversions.convertVector2D(transform.deltaTransform(pointInLocal.getX(), pointInLocal.getY()));
   }

   private static double calculateGridSize(double pixelsPerMeter)
   {
      double medianGridWidthInPixels = Toolkit.getDefaultToolkit().getScreenResolution();
      double desiredMeters = medianGridWidthInPixels / pixelsPerMeter;
      double decimalPlace = Math.log10(desiredMeters);
      double orderOfMagnitude = Math.floor(decimalPlace);
      double nextOrderOfMagnitude = Math.pow(10, orderOfMagnitude + 1);
      double percentageToNextOrderOfMagnitude = desiredMeters / nextOrderOfMagnitude;

      double remainder = percentageToNextOrderOfMagnitude % 0.5;
      double roundToNearestPoint5 = remainder >= 0.25 ? percentageToNextOrderOfMagnitude + (0.5 - remainder) : percentageToNextOrderOfMagnitude - remainder;

      double gridSizeMeters;

      if (roundToNearestPoint5 > 0.0)
      {
         gridSizeMeters = nextOrderOfMagnitude * roundToNearestPoint5;
      }
      else
      {
         gridSizeMeters = Math.pow(10, orderOfMagnitude);
      }

      return gridSizeMeters;
   }

   public Node getRoot()
   {
      return canvas;
   }

   public void setLineWidth(double lineWidth)
   {
      lineWidthProperty.set(lineWidth);
   }

   public double getLineWidth()
   {
      return lineWidthProperty.get();
   }

   public DoubleProperty lineWidthProperty()
   {
      return lineWidthProperty;
   }

   public void setColor(Color color)
   {
      colorProperty.set(color);
   }

   public Color getColor()
   {
      return colorProperty.get();
   }

   public ObjectProperty<Color> colorProperty()
   {
      return colorProperty;
   }

   public void setFont(Font font)
   {
      fontProperty.set(font);
   }

   public Font getFont()
   {
      return fontProperty.get();
   }

   public ObjectProperty<Font> fontProperty()
   {
      return fontProperty;
   }
}
