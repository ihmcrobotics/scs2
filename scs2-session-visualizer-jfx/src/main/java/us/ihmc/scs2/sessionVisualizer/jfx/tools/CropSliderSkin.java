package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import com.jfoenix.controls.JFXSlider.IndicatorPosition;
import com.jfoenix.skins.JFXSliderSkin;

import javafx.animation.Interpolator;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.animation.Transition;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.geometry.Orientation;
import javafx.geometry.Point2D;
import javafx.scene.Node;
import javafx.scene.layout.StackPane;
import javafx.util.Duration;
import us.ihmc.commons.MathTools;

public class CropSliderSkin extends JFXSliderSkin
{
   private static final String ORIENTATION = "ORIENTATION";
   private static final String INDICATOR_POSITION = "INDICATOR_POSITION";
   private static final String TRIM_START_VALUE = "TRIM_START_VALUE";
   private static final String TRIM_END_VALUE = "TRIM_END_VALUE";
   private static final String TRIM_SHOW = "TRIM_SHOW";

   private static final double markerSlopeRatio = 0.45;

   private static final double[] underLeftHorizontalTrim = {0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0, markerSlopeRatio};
   private static final double[] underRightHorizontalTrim = {0.0, 0.0, 1.0, markerSlopeRatio, 1.0, 1.0, 0.0, 1.0};
   private static final double[] aboveLeftHorizontalTrim = {0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0, -markerSlopeRatio};
   private static final double[] aboveRightHorizontalTrim = {0.0, 0.0, 1.0, -markerSlopeRatio, 1.0, -1.0, 0.0, -1.0};

   private static final double[] leftTopVerticalTrim = {0.0, 0.0, -1.0, 0.0, -1.0, -1.0, -markerSlopeRatio, -1.0};
   private static final double[] leftBottomVerticalTrim = {0.0, 0.0, -1.0, 0.0, -1.0, 1.0, -markerSlopeRatio, 1.0};
   private static final double[] rightTopVerticalTrim = {0.0, 0.0, 1.0, 0.0, 1.0, -1.0, markerSlopeRatio, -1.0};
   private static final double[] rightBottomVerticalTrim = {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, markerSlopeRatio, 1.0};

   private StackPane thumb;

   private TrimPolygonMarker trimStart;
   private TrimPolygonMarker trimEnd;

   private Timeline timeline;

   private double thumbWidth;
   private double thumbHeight;
   private double trackStart;
   private double trackLength;
   private boolean isValid = false;

   private Point2D dragStart; // in skin coordinates
   private double preDragTrimMarkerPosition;
   private boolean boundValueToTrimMarker;

   public CropSliderSkin(CropSlider slider)
   {
      super(slider);

      thumb = (StackPane) getSkinnable().lookup(".thumb");

      trimStart = new TrimPolygonMarker(underLeftHorizontalTrim);
      trimStart.getStyleClass().add("trim-start");
      trimStart.setScaleX(0);
      trimStart.setScaleY(0);
      initTrimMarkerListeners(trimStart, slider.trimStartValueProperty(), slider.trimStartValueChangingProperty());

      trimEnd = new TrimPolygonMarker(underRightHorizontalTrim);
      trimEnd.getStyleClass().add("trim-end");
      trimEnd.setScaleX(0);
      trimEnd.setScaleY(0);
      initTrimMarkerListeners(trimEnd, slider.trimEndValueProperty(), slider.trimEndValueChangingProperty());

      initAnimation();

      getChildren().add(1, trimEnd);
      getChildren().add(1, trimStart);

      registerChangeListener(slider.orientationProperty(), ORIENTATION);
      registerChangeListener(slider.indicatorPositionProperty(), INDICATOR_POSITION);
      registerChangeListener(slider.trimStartValueProperty(), TRIM_START_VALUE);
      registerChangeListener(slider.trimStartValueProperty(), TRIM_START_VALUE);
      registerChangeListener(slider.trimEndValueProperty(), TRIM_END_VALUE);
      registerChangeListener(slider.showTrimProperty(), TRIM_SHOW);
   }

   private void initTrimMarkerListeners(Node marker, DoubleProperty valueProperty, BooleanProperty valueChangingProperty)
   {
      marker.setOnMousePressed(e ->
      {
         CropSlider slider = getSlider();
         if (!slider.isFocused())
            slider.requestFocus();
         valueChangingProperty.set(true);
         dragStart = marker.localToParent(e.getX(), e.getY());
         preDragTrimMarkerPosition = normalize(valueProperty.get());
         if (!slider.valueProperty().isBound())
         {
            slider.valueProperty().bind(valueProperty);
            boundValueToTrimMarker = true;
         }
         thumb.fireEvent(e);
      });

      marker.setOnMouseReleased(e ->
      {
         CropSlider slider = getSlider();
         valueChangingProperty.set(false);
         slider.adjustValue(valueProperty.get());
         if (boundValueToTrimMarker)
            slider.valueProperty().unbind();
         boundValueToTrimMarker = false;
         thumb.fireEvent(e);
      });

      marker.setOnMouseDragged(e ->
      {
         Point2D current = marker.localToParent(e.getX(), e.getY());
         double dragPos = (getOrientation() == Orientation.HORIZONTAL) ? current.getX() - dragStart.getX() : -(current.getY() - dragStart.getY());
         double position = preDragTrimMarkerPosition + dragPos / trackLength;
         valueProperty.set(unnormalize(position));
      });
   }

   @Override
   protected void layoutChildren(double x, double y, double w, double h)
   {
      super.layoutChildren(x, y, w, h);

      if (!isValid)
      {
         updateTrimMarkerPosition();
         initAnimation();
         isValid = true;
      }

      // calculate the available space
      thumbWidth = snapSize(thumb.prefWidth(-1));
      thumbHeight = snapSize(thumb.prefHeight(-1));

      if (getSkinnable().getOrientation() == Orientation.HORIZONTAL)
      {
         trackLength = snapSize(w - thumbWidth);
         trackStart = snapPosition(x + (thumbWidth / 2));
      }
      else
      {
         trackLength = snapSize(h - thumbHeight);
         trackStart = snapPosition(y + (thumbHeight / 2));
      }

      double prefWidth = trimStart.prefWidth(-1);
      trimStart.resize(prefWidth, trimStart.prefHeight(prefWidth));
      prefWidth = trimEnd.prefWidth(-1);
      trimEnd.resize(prefWidth, trimEnd.prefHeight(prefWidth));

      positionTrimMarker(false, trimStart, getSlider().trimStartValueProperty());
      positionTrimMarker(false, trimEnd, getSlider().trimEndValueProperty());
   }

   protected void handleControlPropertyChanged(String p)
   {
      switch (p)
      {
         case ORIENTATION:
         case INDICATOR_POSITION:
            updateTrimMarkerPosition();
            initAnimation();
            break;
         case TRIM_START_VALUE:
            positionTrimMarker(false, trimStart, getSlider().trimStartValueProperty());
            break;
         case TRIM_END_VALUE:
            positionTrimMarker(false, trimEnd, getSlider().trimEndValueProperty());
            break;
         case TRIM_SHOW:
            updateTrimMarkerPosition();
            timeline.setRate(getSlider().isShowTrim() ? 1 : -1);
            timeline.play();
            break;
      }
   }

   private void updateTrimMarkerPosition()
   {
      Orientation orientation = getOrientation();
      if (orientation == Orientation.HORIZONTAL)
      {
         if (getIndicatorPosition() == IndicatorPosition.LEFT)
         {
            trimStart.setPoints(underLeftHorizontalTrim);
            trimEnd.setPoints(underRightHorizontalTrim);
         }
         else
         {
            trimStart.setPoints(aboveLeftHorizontalTrim);
            trimEnd.setPoints(aboveRightHorizontalTrim);
         }
      }
      else
      {
         if (getIndicatorPosition() == IndicatorPosition.LEFT)
         {
            trimStart.setPoints(rightBottomVerticalTrim);
            trimEnd.setPoints(rightTopVerticalTrim);
         }
         else
         {
            trimStart.setPoints(leftBottomVerticalTrim);
            trimEnd.setPoints(leftTopVerticalTrim);
         }
      }
   }

   private void positionTrimMarker(boolean animate, Node marker, DoubleProperty markerValue)
   {
      CropSlider slider = getSlider();

      if (slider.getTrimStartValue() > slider.getMax())
         return;// this can happen if we are bound to something
      boolean horizontal = slider.getOrientation() == Orientation.HORIZONTAL;
      double normalizedValue = normalize(markerValue.get());

      final double endX;

      if (horizontal)
         endX = trackStart + (((trackLength * normalizedValue)));
      else
         endX = thumb.getLayoutX();

      final double endY;
      if (horizontal)
         endY = thumb.getLayoutY();
      else
         endY = snappedTopInset() + trackLength * (1.0 - normalizedValue) + thumbHeight / 2.0;

      if (animate)
      {
         // lets animate the marker transition
         final double startX = marker.getLayoutX();
         final double startY = marker.getLayoutY();

         Transition transition = new Transition()
         {
            {
               setCycleDuration(Duration.millis(200));
            }

            @Override
            protected void interpolate(double fraction)
            {
               if (!Double.isNaN(startX))
                  marker.setLayoutX(startX + fraction * (endX - startX));
               if (!Double.isNaN(startY))
                  marker.setLayoutY(startY + fraction * (endY - startY));
            }
         };

         transition.play();
      }
      else
      {
         marker.setLayoutX(endX);
         marker.setLayoutY(endY);
      }
   }

   private void initAnimation()
   {
      double startTrimPos, startTrimNewPos, endTrimPos, endTrimNewPos;
      DoubleProperty startLayoutProperty, endLayoutProperty;

      double gap = getIndicatorPosition() == IndicatorPosition.LEFT ? getTrimGap() : -getTrimGap();
      double factorThumb = getIndicatorPosition() == IndicatorPosition.LEFT ? 1.0 : 0.0;

      if (getOrientation() == Orientation.HORIZONTAL)
      {
         startTrimPos = thumb.getLayoutY();
         startTrimNewPos = startTrimPos + factorThumb * thumb.getHeight() + gap;

         endTrimPos = thumb.getLayoutY();
         endTrimNewPos = endTrimPos + factorThumb * thumb.getHeight() + gap;

         startLayoutProperty = trimStart.translateYProperty();
         endLayoutProperty = trimEnd.translateYProperty();
      }
      else
      {
         startTrimPos = thumb.getLayoutX();
         startTrimNewPos = startTrimPos + factorThumb * thumb.getWidth() + gap;

         endTrimPos = thumb.getLayoutX();
         endTrimNewPos = endTrimPos + factorThumb * thumb.getWidth() + gap;

         startLayoutProperty = trimStart.translateXProperty();
         endLayoutProperty = trimEnd.translateXProperty();
      }

      Timeline newTimeline = new Timeline(new KeyFrame(Duration.ZERO,
                                                       new KeyValue(trimStart.scaleXProperty(), 0, Interpolator.EASE_BOTH),
                                                       new KeyValue(trimStart.scaleYProperty(), 0, Interpolator.EASE_BOTH),
                                                       new KeyValue(startLayoutProperty, startTrimPos, Interpolator.EASE_BOTH),
                                                       new KeyValue(trimEnd.scaleXProperty(), 0, Interpolator.EASE_BOTH),
                                                       new KeyValue(trimEnd.scaleYProperty(), 0, Interpolator.EASE_BOTH),
                                                       new KeyValue(endLayoutProperty, endTrimPos, Interpolator.EASE_BOTH)),
                                          new KeyFrame(Duration.seconds(0.2),
                                                       new KeyValue(trimStart.scaleXProperty(), 1, Interpolator.EASE_BOTH),
                                                       new KeyValue(trimStart.scaleYProperty(), 1, Interpolator.EASE_BOTH),
                                                       new KeyValue(startLayoutProperty, startTrimNewPos, Interpolator.EASE_BOTH),
                                                       new KeyValue(trimEnd.scaleXProperty(), 1, Interpolator.EASE_BOTH),
                                                       new KeyValue(trimEnd.scaleYProperty(), 1, Interpolator.EASE_BOTH),
                                                       new KeyValue(endLayoutProperty, endTrimNewPos, Interpolator.EASE_BOTH)));

      clearAnimation();
      timeline = newTimeline;
   }

   private double unnormalize(double normalizedValue)
   {
      double value = (normalizedValue * (getSlider().getMax() - getSlider().getMin())) + getSlider().getMin();
      return MathTools.clamp(value, getSlider().getMin(), getSlider().getMax());
   }

   private double normalize(double value)
   {
      return (value - getSlider().getMin()) / (getSlider().getMax() - getSlider().getMin());
   }

   private Orientation getOrientation()
   {
      return getSlider().getOrientation();
   }

   private IndicatorPosition getIndicatorPosition()
   {
      return getSlider().getIndicatorPosition();
   }

   private double getTrimGap()
   {
      return getSlider().getTrimGap();
   }

   private CropSlider getSlider()
   {
      return (CropSlider) getSkinnable();
   }

   @Override
   public void dispose()
   {
      super.dispose();
      clearAnimation();
   }

   private void clearAnimation()
   {
      if (timeline != null)
      {
         timeline.stop();
         timeline.getKeyFrames().clear();
         timeline = null;
      }
   }
}
