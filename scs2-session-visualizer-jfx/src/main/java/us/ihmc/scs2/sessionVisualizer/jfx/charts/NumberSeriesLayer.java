package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.Shape;
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
   /** Above this fraction of the buffer capacity, an incremental redraw isn't worth it over just redrawing everything. */
   private static final double MAX_INCREMENTAL_SPAN_FRACTION = 0.25;
   private static final int MIN_INCREMENTAL_SPAN_FLOOR = 32;

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
   /** Set only when the X/Y axis bounds themselves change (pan/zoom/rescale), as opposed to other layout-affecting changes. */
   private final BooleanProperty axisBoundsChangedProperty = new SimpleBooleanProperty(this, "axisBoundsChanged", false);
   /** Set only when the stroke color/width changes, which invalidates every already-drawn pixel (unlike a plain data update). */
   private final BooleanProperty styleChangedProperty = new SimpleBooleanProperty(this, "styleChanged", false);
   /** Region of {@link #imageToRender} that actually changed in the last {@link #updateImage()} call, consumed by {@link #render()}. */
   private volatile Rectangle2D pendingDirtyRegion = null;
   /** Buffer write-index as of the last redraw; {@code -1} means never drawn, which forces a full redraw. */
   private int lastDrawnCurrentIndex = -1;
   /** {@code data.size()} as of the last redraw; used to detect a buffer resize/crop even when it doesn't trip {@link #axisBoundsChangedProperty}. */
   private int lastDrawnDataSize = -1;
   /** {@code numberSeries.isNegated()} as of the last redraw; a change invalidates previously-drawn pixel positions even in RAW style. */
   private boolean lastDrawnNegated = false;

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
      InvalidationListener axisBoundsDirtyListener = (InvalidationListener) ->
      {
         layoutChangedProperty.set(true);
         axisBoundsChangedProperty.set(true);
      };

      ChangeListener<? super FastAxisBase> axisChangeListener = (o, oldAxis, newAxis) ->
      {
         if (oldAxis != null)
         {
            oldAxis.lowerBoundProperty().removeListener(axisBoundsDirtyListener);
            oldAxis.upperBoundProperty().removeListener(axisBoundsDirtyListener);
         }
         newAxis.lowerBoundProperty().addListener(axisBoundsDirtyListener);
         newAxis.upperBoundProperty().addListener(axisBoundsDirtyListener);
      };
      xAxis.addListener(axisChangeListener);
      yAxis.addListener(axisChangeListener);
      axisChangeListener.changed(null, null, xAxis.get());
      axisChangeListener.changed(null, null, yAxis.get());

      InvalidationListener styleChangedListener = (InvalidationListener) ->
      {
         layoutChangedProperty.set(true);
         styleChangedProperty.set(true);
      };
      stroke.addListener(styleChangedListener);
      strokeWidth.addListener(styleChangedListener);
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

      Rectangle2D dirtyRegion = pendingDirtyRegion != null ? pendingDirtyRegion : new Rectangle2D(0, 0, width, height);
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

      boolean useFullRedraw = !clearImage;

      List<Point2D> data = numberSeries.getData();
      dataSizeProperty.set(data.size());

      numberSeries.getLock().readLock().lock();

      try
      {
         if (data.isEmpty())
            return false;

         boolean axisBoundsChanged = pollAxisBoundsChanged();
         boolean styleChanged = pollStyleChanged();
         // NORMALIZED's Y-transform depends on customYBounds/yBoundsProperty, which can change via a plain data update
         // without tripping any of the flags above; always redrawing fully for NORMALIZED sidesteps that instead of
         // tracking it separately. Negation flips the Y-transform too, in both chart styles, so it needs the same
         // treatment as an axis rescale: previously-drawn pixels are no longer valid, not just "unchanged".
         useFullRedraw = useFullRedraw || axisBoundsChanged || styleChanged || chartStyleProperty.get() == ChartStyle.NORMALIZED
                         || numberSeries.isNegated() != lastDrawnNegated || data.size() != lastDrawnDataSize;

         if (!numberSeries.pollDirty() && !pollLayoutChanged())
            return false;

         xData = resize(xData, data.size());
         yData = resize(yData, data.size());

         int[] dirtyBounds = {Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE};

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

         int newCurrentIndex = numberSeries.bufferCurrentIndexProperty().get();
         boolean currentIndexInBounds = newCurrentIndex >= 0 && newCurrentIndex < data.size();

         IndexRange[] changedRanges = null;
         if (!useFullRedraw && currentIndexInBounds)
         {
            int maxIncrementalSpan = Math.max(MIN_INCREMENTAL_SPAN_FLOOR, (int) (data.size() * MAX_INCREMENTAL_SPAN_FRACTION));
            changedRanges = computeChangedIndexRanges(lastDrawnCurrentIndex, newCurrentIndex, data.size(), maxIncrementalSpan);
         }

         if (changedRanges != null)
         {
            // Oscilloscope-style live view: the X-axis is pinned to [0, bufferSize) and never scrolls (confirmed via
            // YoChartPanelController), so only the ring-buffer index range between the last write position and the new
            // one actually changed value. Redrawing (and reporting dirty) just that range is what makes this cheap.
            for (IndexRange range : changedRanges)
               drawIndexRangeIncrementally(graphics, data, xTransform, yTransform, xData, yData, range, strokeWidth.get(), widthInt, heightInt, dirtyBounds);
         }
         else
         {
            useFullRedraw = true;
            if (clearImage)
               graphics.clearRect(0, 0, widthInt, heightInt);
            drawMultiLine(graphics, data, xTransform, yTransform, xData, yData);
            accumulateDirtyBounds(xData, yData, data.size(), dirtyBounds);
         }

         if (updateIndexMarkerVisible.get())
         {
            // No special-casing needed for the marker's old position: it always sits at bufferCurrentIndexProperty, i.e.
            // exactly one endpoint of the redrawn range above (incremental or full), so it's already been erased there.
            graphics.setColor(toAWTColor(Color.GREY.deriveColor(0, 1.0, 0.92, 0.5)));
            graphics.setStroke(new BasicStroke(1.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_MITER));
            List<Point2D> markerData = Arrays.asList(new Point2D(numberSeries.bufferCurrentIndexProperty().get(), yAxis.get().getLowerBound()),
                                                     new Point2D(numberSeries.bufferCurrentIndexProperty().get(), yAxis.get().getUpperBound()));
            drawMultiLine(graphics, markerData, xTransform, yTransform, xData, yData);
            accumulateDirtyBounds(xData, yData, markerData.size(), dirtyBounds);
         }

         pendingDirtyRegion = computeDirtyRegion(useFullRedraw, dirtyBounds, strokeWidth.get(), widthInt, heightInt);

         lastDrawnCurrentIndex = currentIndexInBounds ? newCurrentIndex : lastDrawnCurrentIndex;
         lastDrawnDataSize = data.size();
         lastDrawnNegated = numberSeries.isNegated();

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

   private boolean pollAxisBoundsChanged()
   {
      boolean ret = axisBoundsChangedProperty.get();
      axisBoundsChangedProperty.set(false);
      return ret;
   }

   private boolean pollStyleChanged()
   {
      boolean ret = styleChangedProperty.get();
      styleChangedProperty.set(false);
      return ret;
   }

   /**
    * Folds the pixel coordinates just written to {@code xData}/{@code yData} (indices {@code 0} to {@code n - 1}) into the
    * running dirty-region bounds {@code boundsInOut}, laid out as {@code {minX, minY, maxX, maxY}}. Package-private and
    * static so it can be unit tested without constructing a {@link NumberSeriesLayer} or starting the JavaFX toolkit.
    */
   static void accumulateDirtyBounds(int[] xData, int[] yData, int n, int[] boundsInOut)
   {
      for (int i = 0; i < n; i++)
      {
         int x = xData[i];
         int y = yData[i];
         if (x < boundsInOut[0])
            boundsInOut[0] = x;
         if (x > boundsInOut[2])
            boundsInOut[2] = x;
         if (y < boundsInOut[1])
            boundsInOut[1] = y;
         if (y > boundsInOut[3])
            boundsInOut[3] = y;
      }
   }

   /**
    * Reports the whole image as dirty when a full redraw was required (resize or axis rescale), otherwise reports just the
    * bounding box of what was actually drawn this update ({@code bounds}, laid out as {@code {minX, minY, maxX, maxY}}),
    * padded to cover stroke width/anti-aliasing bleed. Package-private and static for the same testability reason as
    * {@link #accumulateDirtyBounds}.
    */
   static Rectangle2D computeDirtyRegion(boolean useFullDirtyRect, int[] bounds, double strokeWidth, int widthInt, int heightInt)
   {
      if (useFullDirtyRect || bounds[0] > bounds[2])
         return new Rectangle2D(0, 0, widthInt, heightInt);

      int margin = Math.max(2, (int) Math.ceil(strokeWidth));
      int minX = Math.max(0, bounds[0] - margin);
      int minY = Math.max(0, bounds[1] - margin);
      int maxX = Math.min(widthInt, bounds[2] + margin);
      int maxY = Math.min(heightInt, bounds[3] + margin);
      return new Rectangle2D(minX, minY, Math.max(1, maxX - minX), Math.max(1, maxY - minY));
   }

   /** A closed data-index range {@code [from, to]}, both inclusive, into {@link NumberSeries#getData()}. */
   static final class IndexRange
   {
      final int from;
      final int to;

      IndexRange(int from, int to)
      {
         this.from = from;
         this.to = to;
      }

      @Override
      public boolean equals(Object obj)
      {
         if (this == obj)
            return true;
         if (!(obj instanceof IndexRange))
            return false;
         IndexRange other = (IndexRange) obj;
         return from == other.from && to == other.to;
      }

      @Override
      public int hashCode()
      {
         return 31 * from + to;
      }

      @Override
      public String toString()
      {
         return "IndexRange[" + from + ", " + to + "]";
      }
   }

   /**
    * Computes which data-index range(s) actually changed between two buffer write positions, ring-buffer-wraparound aware,
    * padded by one index on each side so the redrawn segment reconnects correctly to the still-valid surrounding line.
    * Returns {@code null} (meaning "do a full redraw instead") when there's nothing to diff against yet ({@code oldIndex <
    * 0}), either index is out of bounds (stale relative to a resized/cropped buffer), or the changed span exceeds
    * {@code maxIncrementalSpan} (large jumps, e.g. scrubbing or resuming after a pause, aren't worth doing incrementally).
    * Package-private and static so it can be unit tested without constructing a {@link NumberSeriesLayer} or the JavaFX
    * toolkit, same as {@link #computeDirtyRegion}.
    */
   static IndexRange[] computeChangedIndexRanges(int oldIndex, int newIndex, int bufferSize, int maxIncrementalSpan)
   {
      if (bufferSize <= 0 || oldIndex < 0 || oldIndex >= bufferSize || newIndex < 0 || newIndex >= bufferSize)
         return null;

      if (newIndex >= oldIndex)
      {
         int span = newIndex - oldIndex;
         if (span > maxIncrementalSpan)
            return null;
         return new IndexRange[] {new IndexRange(Math.max(0, oldIndex - 1), Math.min(bufferSize - 1, newIndex + 1))};
      }
      else
      {
         int span = (bufferSize - oldIndex) + newIndex;
         if (span > maxIncrementalSpan)
            return null;
         return new IndexRange[] {new IndexRange(Math.max(0, oldIndex - 1), bufferSize - 1),
                                   new IndexRange(0, Math.min(bufferSize - 1, newIndex + 1))};
      }
   }

   /**
    * Clears and redraws just the pixel span covering {@code range}, via an AWT clip so the redraw can't bleed into
    * still-valid pixels outside it, and folds the drawn pixels into {@code dirtyBoundsInOut}.
    */
   private static void drawIndexRangeIncrementally(Graphics2D graphics,
                                                    List<Point2D> data,
                                                    DoubleUnaryOperator xTransform,
                                                    DoubleUnaryOperator yTransform,
                                                    int[] xData,
                                                    int[] yData,
                                                    IndexRange range,
                                                    double strokeWidth,
                                                    int widthInt,
                                                    int heightInt,
                                                    int[] dirtyBoundsInOut)
   {
      int count = range.to - range.from + 1;

      int margin = Math.max(2, (int) Math.ceil(strokeWidth));
      int pixelXStart = (int) Math.floor(xTransform.applyAsDouble(range.from));
      int pixelXEnd = (int) Math.ceil(xTransform.applyAsDouble(range.to));
      int clipMinX = Math.max(0, Math.min(pixelXStart, pixelXEnd) - margin);
      int clipMaxX = Math.min(widthInt, Math.max(pixelXStart, pixelXEnd) + margin);
      int clipWidth = Math.max(1, clipMaxX - clipMinX);

      Shape previousClip = graphics.getClip();
      graphics.setClip(clipMinX, 0, clipWidth, heightInt);
      graphics.clearRect(clipMinX, 0, clipWidth, heightInt);

      for (int i = 0; i < count; i++)
      {
         Point2D point = data.get(range.from + i);
         xData[i] = (int) Math.round(xTransform.applyAsDouble(point.getX()));
         yData[i] = (int) Math.round(yTransform.applyAsDouble(point.getY()));
      }
      graphics.drawPolyline(xData, yData, count);
      accumulateDirtyBounds(xData, yData, count, dirtyBoundsInOut);

      graphics.setClip(previousClip);
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