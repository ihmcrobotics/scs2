package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.nio.IntBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleUnaryOperator;

import javafx.beans.InvalidationListener;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.css.CssMetaData;
import javafx.css.Styleable;
import javafx.css.StyleableDoubleProperty;
import javafx.css.StyleableObjectProperty;
import javafx.css.StyleablePropertyFactory;
import javafx.geometry.Rectangle2D;
import javafx.scene.chart.FastAxisBase;
import javafx.scene.image.ImageView;
import javafx.scene.image.PixelBuffer;
import javafx.scene.image.PixelFormat;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.DynamicLineChart.ChartStyle;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ChartRenderManager;

public class NumberSeriesLayer extends ImageView
{
   private static final StyleablePropertyFactory<NumberSeriesLayer> FACTORY = new StyleablePropertyFactory<>(NumberSeriesLayer.getClassCssMetaData());
   private static final CssMetaData<NumberSeriesLayer, Color> STROKE = FACTORY.createColorCssMetaData("-fx-stroke", s -> s.stroke, Color.BLUE, false);
   private static final CssMetaData<NumberSeriesLayer, Number> STROKE_WIDTH = FACTORY.createSizeCssMetaData("-fx-stroke-width", s -> s.strokeWidth, 1.0, false);

   private final NumberSeries numberSeries;
   private final DynamicChartLegendItem legendNode = new DynamicChartLegendItem();

   protected final ObjectProperty<FastAxisBase> xAxis;
   protected final ObjectProperty<FastAxisBase> yAxis;

   private final BooleanProperty layoutChangedProperty = new SimpleBooleanProperty(this, "layoutChanged", true);

   private final Executor backgroundExecutor;

   private final StyleableObjectProperty<Color> stroke = new StyleableObjectProperty<Color>(Color.BLACK)
   {
      @Override
      protected void invalidated()
      {
         Paint color = get();
         scheduleRender();
         legendNode.setTextFill(color);
      };

      @Override
      public String getName()
      {
         return "stroke";
      }

      @Override
      public Object getBean()
      {
         return NumberSeriesLayer.this;
      }

      @Override
      public CssMetaData<? extends Styleable, Color> getCssMetaData()
      {
         return STROKE;
      }
   };

   private final StyleableDoubleProperty strokeWidth = new StyleableDoubleProperty(1.0)
   {
      @Override
      protected void invalidated()
      {
         scheduleRender();
      };

      @Override
      public String getName()
      {
         return "strokeWidth";
      }

      @Override
      public Object getBean()
      {
         return NumberSeriesLayer.this;
      }

      @Override
      public CssMetaData<? extends Styleable, Number> getCssMetaData()
      {
         return STROKE_WIDTH;
      }
   };

   private final ChartRenderManager renderManager;

   private final ObjectProperty<ChartStyle> chartStyleProperty = new SimpleObjectProperty<>(this, "chartStyle", ChartStyle.RAW);
   private PixelBuffer<IntBuffer> pixelBuffer = null;
   private AtomicBoolean renderNewImage = new AtomicBoolean(true);
   private AtomicBoolean isRenderingImage = new AtomicBoolean(false);
   private AtomicBoolean isUpdatingImage = new AtomicBoolean(false);
   private BufferedImage imageToRender = null;
   private IntegerProperty dataSizeProperty = new SimpleIntegerProperty(this, "dataSize", 0);
   private BooleanProperty updateIndexMarkerVisible = new SimpleBooleanProperty(this, "updateIndexMarkerVisible", false);

   public NumberSeriesLayer(ObjectProperty<FastAxisBase> xAxis,
                            ObjectProperty<FastAxisBase> yAxis,
                            NumberSeries numberSeries,
                            Executor backgroundExecutor,
                            ChartRenderManager renderManager)
   {
      this.renderManager = renderManager;
      getStyleClass().add("dynamic-chart-series-line");
      this.xAxis = xAxis;
      this.yAxis = yAxis;
      this.numberSeries = numberSeries;
      this.backgroundExecutor = backgroundExecutor;
      legendNode.seriesNameProperty().bind(numberSeries.seriesNameProperty());
      legendNode.currentValueProperty().bind(numberSeries.currentValueProperty());

      InvalidationListener dirtyListener = (InvalidationListener) -> layoutChangedProperty.set(true);

      ChangeListener<? super FastAxisBase> axisChangeListener = (o, oldAxis, newAxis) ->
      {
         if (oldAxis != null)
         {
            oldAxis.lowerBoundProperty().removeListener(dirtyListener);
            oldAxis.upperBoundProperty().removeListener(dirtyListener);
         }
         newAxis.lowerBoundProperty().addListener(dirtyListener);
         newAxis.upperBoundProperty().addListener(dirtyListener);
      };
      xAxis.addListener(axisChangeListener);
      yAxis.addListener(axisChangeListener);
      axisChangeListener.changed(null, null, xAxis.get());
      axisChangeListener.changed(null, null, yAxis.get());

      stroke.addListener(dirtyListener);
      strokeWidth.addListener(dirtyListener);
      dataSizeProperty.addListener(dirtyListener);
      updateIndexMarkerVisible.addListener(dirtyListener);
   }

   public void scheduleRender()
   {
      backgroundExecutor.execute(() ->
      {
         if (updateImage())
            renderManager.submitRenderRequest(this::render);
      });
   }

   private void render()
   {
      if (imageToRender == null)
         return;

      if (isUpdatingImage.get())
      {
         renderManager.submitRenderRequest(this::render);
         return;
      }

      isRenderingImage.set(true);

      int width = imageToRender.getWidth();
      int height = imageToRender.getHeight();

      if (renderNewImage.getAndSet(false))
      {
         pixelBuffer = new PixelBuffer<>(width,
                                         height,
                                         IntBuffer.wrap(((DataBufferInt) imageToRender.getRaster().getDataBuffer()).getData()),
                                         PixelFormat.getIntArgbPreInstance());
         setImage(new WritableImage(pixelBuffer));
      }

      Rectangle2D dirtyRegion = new Rectangle2D(0, 0, width, height);
      pixelBuffer.updateBuffer(b -> dirtyRegion);

      isRenderingImage.set(false);
   }

   private int[] xData, yData;
   private Graphics2D graphics;

   private boolean updateImage()
   {
      if (isRenderingImage.get())
         return false;

      if (isUpdatingImage.get())
         return false;

      double width = xAxis.get().getWidth();
      double height = yAxis.get().getHeight();
      int widthInt = (int) Math.round(width);
      int heightInt = (int) Math.round(height);

      if (widthInt == 0 || heightInt == 0)
         return false;

      isUpdatingImage.set(true);

      boolean clearImage = true;

      if (imageToRender == null || imageToRender.getWidth() != widthInt || imageToRender.getHeight() != heightInt)
      {
         layoutChangedProperty.set(true);
         imageToRender = new BufferedImage(widthInt, heightInt, BufferedImage.TYPE_INT_ARGB_PRE);
         graphics = imageToRender.createGraphics();
         graphics.setBackground(new java.awt.Color(255, 255, 255, 0));
         renderNewImage.set(true);
         clearImage = false;
      }

      List<Point2D> data = numberSeries.getData();
      dataSizeProperty.set(data.size());

      numberSeries.getLock().readLock().lock();

      try
      {
         if (data.isEmpty())
            return false;
         else if (!numberSeries.pollDirty() && !pollLayoutChanged())
            return false;

         xData = resize(xData, data.size());
         yData = resize(yData, data.size());

         if (clearImage)
            graphics.clearRect(0, 0, widthInt, heightInt);

         graphics.setColor(toAWTColor(stroke.get()));
         graphics.setStroke(new BasicStroke((float) (strokeWidth.get()), BasicStroke.CAP_ROUND, BasicStroke.JOIN_MITER));

         DoubleUnaryOperator xTransform = xToHorizontalDisplayTransform(width, xAxis.get().getLowerBound(), xAxis.get().getUpperBound());
         DoubleUnaryOperator yTransform = yToVerticalDisplayTransform(height, yAxis.get().getLowerBound(), yAxis.get().getUpperBound());

         if (chartStyleProperty.get() == ChartStyle.NORMALIZED)
         {
            ChartDoubleBounds yBounds = numberSeries.getCustomYBounds();
            if (yBounds == null)
               yBounds = numberSeries.yBoundsProperty().getValue();
            if (numberSeries.isNegated())
               yBounds = yBounds.negate();

            yTransform = yTransform.compose(normalizeTransform(yBounds));
         }
         if (numberSeries.isNegated())
         {
            yTransform = yTransform.compose(negateTransform());
         }

         drawMultiLine(graphics, data, xTransform, yTransform, xData, yData);

         if (updateIndexMarkerVisible.get())
         {
            graphics.setColor(toAWTColor(Color.GREY.deriveColor(0, 1.0, 0.92, 0.5)));
            graphics.setStroke(new BasicStroke(1.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_MITER));
            List<Point2D> markerData = Arrays.asList(new Point2D(numberSeries.bufferCurrentIndexProperty().get(), yAxis.get().getLowerBound()),
                                                     new Point2D(numberSeries.bufferCurrentIndexProperty().get(), yAxis.get().getUpperBound()));
            drawMultiLine(graphics, markerData, xTransform, yTransform, xData, yData);
         }

         return true;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return false;
      }
      finally
      {
         isUpdatingImage.set(false);
         numberSeries.getLock().readLock().unlock();
      }
   }

   private boolean pollLayoutChanged()
   {
      boolean ret = layoutChangedProperty.get();
      layoutChangedProperty.set(false);
      return ret;
   }

   private static int[] resize(int[] in, int length)
   {
      if (in == null || in.length < length)
         return new int[length];
      else
         return in;
   }

   private static void drawMultiLine(Graphics2D graphics,
                                     List<Point2D> points,
                                     DoubleUnaryOperator xTransform,
                                     DoubleUnaryOperator yTransform,
                                     int[] xData,
                                     int[] yData)
   {
      for (int i = 0; i < points.size(); i++)
      {
         Point2D point = points.get(i);
         xData[i] = (int) Math.round(xTransform.applyAsDouble(point.getX()));
         yData[i] = (int) Math.round(yTransform.applyAsDouble(point.getY()));
      }
      graphics.drawPolyline(xData, yData, points.size());
   }

   private static java.awt.Color toAWTColor(Color color)
   {
      float red = (float) color.getRed();
      float green = (float) color.getGreen();
      float blue = (float) color.getBlue();
      float alpha = (float) color.getOpacity();
      return new java.awt.Color(red, green, blue, alpha);
   }

   private static DoubleUnaryOperator negateTransform()
   {
      return coordinate -> -coordinate;
   }

   private static DoubleUnaryOperator normalizeTransform(ChartDoubleBounds bounds)
   {
      return normalizeTransform(bounds.getLower(), bounds.getUpper());
   }

   private static DoubleUnaryOperator normalizeTransform(double min, double max)
   {
      if (min == max)
         return coordinate -> 0.5;

      double invRange = 1.0 / (max - min);

      if (Double.isInfinite(invRange))
         return coordinate -> 0.5;

      return affineTransform(invRange, -min * invRange);
   }

   private static DoubleUnaryOperator xToHorizontalDisplayTransform(double displayWidth, double xMin, double xMax)
   {
      if (xMax == xMin)
         return affineTransform(displayWidth, -xMin * displayWidth);

      double invRange = 1.0 / (xMax - xMin);

      if (Double.isInfinite(invRange))
         return affineTransform(displayWidth, -xMin * displayWidth);

      return affineTransform(displayWidth * invRange, -xMin * displayWidth * invRange);
   }

   private static DoubleUnaryOperator yToVerticalDisplayTransform(double displayHeight, double yMin, double yMax)
   {
      if (yMax == yMin)
         return affineTransform(-displayHeight, displayHeight * (1.0 + yMin));

      double invRange = 1.0 / (yMax - yMin);

      if (Double.isInfinite(invRange))
         return affineTransform(-displayHeight, displayHeight * (1.0 + yMin));

      return affineTransform(-displayHeight * invRange, displayHeight * (1.0 + yMin * invRange));
   }

   private static DoubleUnaryOperator affineTransform(double scale, double offset)
   {
      return coordinate -> coordinate * scale + offset;
   }

   public BooleanProperty updateIndexMarkerVisibleProperty()
   {
      return updateIndexMarkerVisible;
   }

   public ObjectProperty<ChartStyle> chartStyleProperty()
   {
      return chartStyleProperty;
   }

   public NumberSeries getNumberSeries()
   {
      return numberSeries;
   }

   public DynamicChartLegendItem getLegendNode()
   {
      return legendNode;
   }

   @Override
   public List<CssMetaData<? extends Styleable, ?>> getCssMetaData()
   {
      return FACTORY.getCssMetaData();
   }
}