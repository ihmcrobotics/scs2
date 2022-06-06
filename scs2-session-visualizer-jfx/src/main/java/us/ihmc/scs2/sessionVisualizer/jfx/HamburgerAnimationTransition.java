/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

package us.ihmc.scs2.sessionVisualizer.jfx;

import java.util.ArrayList;
import java.util.List;

import com.jfoenix.controls.JFXHamburger;
import com.jfoenix.transitions.CachedTransition;
import com.jfoenix.transitions.hamburger.HamburgerTransition;

import javafx.animation.Interpolator;
import javafx.animation.KeyFrame;
import javafx.animation.KeyValue;
import javafx.animation.Timeline;
import javafx.animation.Transition;
import javafx.application.Platform;
import javafx.beans.binding.Bindings;
import javafx.beans.value.WritableValue;
import javafx.scene.Node;
import javafx.scene.layout.Region;
import javafx.util.Duration;
import us.ihmc.euclid.tools.EuclidCoreTools;

public class HamburgerAnimationTransition extends CachedTransition implements HamburgerTransition
{
   private static final Interpolator interpolation = Interpolator.EASE_BOTH;

   public enum FrameType
   {
      BURGER, LEFT_ARROW, RIGHT_ARROW, LEFT_SHORT_ARROW, RIGHT_SHORT_ARROW, LEFT_ANGLE, RIGHT_ANGLE, LEFT_CLOSE, RIGHT_CLOSE,
   };

   private final Duration startTime = Duration.ZERO;
   private final Duration finalTime = Duration.millis(1000);

   private final JFXHamburger burger;
   private final Node line0;
   private final Node line1;
   private final Node line2;

   public HamburgerAnimationTransition(JFXHamburger burger)
   {
      this(burger, FrameType.LEFT_SHORT_ARROW, FrameType.RIGHT_ANGLE);
   }

   public HamburgerAnimationTransition(JFXHamburger burger, FrameType startType, FrameType finalType)
   {
      super(burger, null);
      this.burger = burger;

      line0 = burger.getChildren().get(0);
      line1 = burger.getChildren().get(1);
      line2 = burger.getChildren().get(2);

      timeline.set(createTimeline(startType, finalType));
      timeline.bind(Bindings.createObjectBinding(() -> createTimeline(startType, finalType),
                                                 burger.widthProperty(),
                                                 burger.heightProperty(),
                                                 ((Region) line0).widthProperty(),
                                                 ((Region) line0).heightProperty()));
      // reduce the number to increase the shifting , increase number to reduce shifting
      setCycleDuration(Duration.seconds(0.5));
      setDelay(Duration.seconds(0));
      initializeFrameType();
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public void initializeFrameType()
   {
      Platform.runLater(() ->
      {
         for (KeyValue keyValue : timeline.get().getKeyFrames().get(0).getValues())
         {
            ((WritableValue) keyValue.getTarget()).setValue(keyValue.getEndValue());
         }
      });
   }

   private Timeline createTimeline(FrameType startType, FrameType finalType)
   {
      return new Timeline(keyFrame(startTime, startType), keyFrame(finalTime, finalType));
   }

   private KeyFrame keyFrame(Duration time, FrameType type)
   {
      switch (type)
      {
         case BURGER:
            return new KeyFrame(time, defaultKeyValues());
         case LEFT_ARROW:
            return new KeyFrame(time, arrowKeyValues(false, false));
         case RIGHT_ARROW:
            return new KeyFrame(time, arrowKeyValues(true, false));
         case LEFT_SHORT_ARROW:
            return new KeyFrame(time, arrowKeyValues(false, true));
         case RIGHT_SHORT_ARROW:
            return new KeyFrame(time, arrowKeyValues(true, true));
         case LEFT_ANGLE:
            return new KeyFrame(time, angleKeyValues(false, true));
         case RIGHT_ANGLE:
            return new KeyFrame(time, angleKeyValues(true, true));
         case LEFT_CLOSE:
            return new KeyFrame(time, closeKeyValues(false));
         case RIGHT_CLOSE:
            return new KeyFrame(time, closeKeyValues(true));
         default:
            throw new RuntimeException();
      }
   }

   private KeyValue[] defaultKeyValues()
   {
      List<KeyValue> keyValues = new ArrayList<>();

      for (Node node : new Node[] {burger, line0, line1, line2})
      {
         keyValues.add(new KeyValue(node.rotateProperty(), 0, interpolation));
         keyValues.add(new KeyValue(node.translateXProperty(), 0, interpolation));
         keyValues.add(new KeyValue(node.translateYProperty(), 0, interpolation));
         keyValues.add(new KeyValue(node.scaleXProperty(), 1, interpolation));
         keyValues.add(new KeyValue(node.scaleYProperty(), 1, interpolation));
         keyValues.add(new KeyValue(node.opacityProperty(), 1, interpolation));
      }

      return keyValues.toArray(new KeyValue[keyValues.size()]);
   }

   private KeyValue[] arrowKeyValues(boolean rightArrow, boolean shortArrow)
   {
      double line0Height = line0.getLayoutBounds().getHeight();
      double line0HalfHeight = line0Height / 2;
      double line0Width = line0.getBoundsInParent().getWidth();
      double line0HalfWidth = line0Width / 2;

      double burgerWidth = line0.getLayoutBounds().getWidth();
      double burgerHalfWidth = burgerWidth / 2;
      double burgerHeight = line2.getBoundsInParent().getMaxY() - line0.getBoundsInParent().getMinY();
      double burgerHalfHeight = burgerHeight / 2;

      double hypotenuse = EuclidCoreTools.norm(burgerHalfHeight - line0HalfHeight, burgerHalfWidth);
      double angle = Math.asin((burgerHalfHeight - line0HalfHeight) / hypotenuse);
      double angleDegrees = Math.toDegrees(angle);

      double burgerDiagonal = EuclidCoreTools.norm(line0Height, line0HalfWidth);
      double theta = (0.5 * Math.PI - angle) + Math.atan(line0Height / line0HalfWidth);
      double hOffset = Math.cos(theta) * burgerDiagonal / 2;
      double line1_scaleX;
      double line1_translationX;
      double translationX;
      if (shortArrow)
      {
         line1_scaleX = 0.6;
         line1_translationX = -0.1 * burgerHalfWidth;
         translationX = 0.5 * burgerHalfWidth - Math.sin(theta) * (burgerDiagonal / 2);
      }
      else
      {
         line1_scaleX = 1.0;
         line1_translationX = 0;
         translationX = burgerHalfWidth - Math.sin(theta) * (burgerDiagonal / 2);
      }

      double translationY = line0HalfHeight + burger.getSpacing() - hOffset;

      if (!rightArrow)
      {
         angleDegrees = -angleDegrees;
         line1_translationX = -line1_translationX;
         translationX = -translationX;
      }

      return new KeyValue[] {new KeyValue(burger.rotateProperty(), 0, interpolation),
                             new KeyValue(line0.rotateProperty(), angleDegrees, interpolation),
                             new KeyValue(line0.translateXProperty(), translationX, interpolation),
                             new KeyValue(line0.translateYProperty(), translationY, interpolation),
                             new KeyValue(line0.scaleXProperty(), 0.5, interpolation),
                             new KeyValue(line0.opacityProperty(), 1, interpolation),
                             new KeyValue(line1.translateXProperty(), line1_translationX, interpolation),
                             new KeyValue(line1.translateYProperty(), 0, interpolation),
                             new KeyValue(line1.scaleXProperty(), line1_scaleX, interpolation),
                             new KeyValue(line1.opacityProperty(), 1, interpolation),
                             new KeyValue(line2.rotateProperty(), -angleDegrees, interpolation),
                             new KeyValue(line2.translateXProperty(), translationX, interpolation),
                             new KeyValue(line2.translateYProperty(), -translationY, interpolation),
                             new KeyValue(line2.scaleXProperty(), 0.5, interpolation),
                             new KeyValue(line2.opacityProperty(), 1, interpolation)};
   }

   private KeyValue[] angleKeyValues(boolean rightAngle, boolean shortArrow)
   {
      double line0Height = line0.getLayoutBounds().getHeight();
      double line0HalfHeight = line0Height / 2;
      double line0Width = line0.getBoundsInParent().getWidth();
      double line0HalfWidth = line0Width / 2;

      double burgerWidth = line0.getLayoutBounds().getWidth();
      double burgerHalfWidth = burgerWidth / 2;
      double burgerHeight = line2.getBoundsInParent().getMaxY() - line0.getBoundsInParent().getMinY();
      double burgerHalfHeight = burgerHeight / 2;

      double hypotenuse = EuclidCoreTools.norm(burgerHalfHeight - line0HalfHeight, burgerHalfWidth);
      double angle = Math.asin((burgerHalfHeight - line0HalfHeight) / hypotenuse);
      double angleDegrees = Math.toDegrees(angle);

      double burgerDiagonal = EuclidCoreTools.norm(line0Height, line0HalfWidth);
      double theta = (0.5 * Math.PI - angle) + Math.atan(line0Height / line0HalfWidth);
      double hOffset = Math.cos(theta) * burgerDiagonal / 2;
      double line1_translationX = -burger.getWidth() / 1.1;
      double translationX;
      if (shortArrow)
      {
         translationX = 0.5 * burgerHalfWidth - Math.sin(theta) * (burgerDiagonal / 2);
      }
      else
      {
         line1_translationX = 0;
         translationX = burgerHalfWidth - Math.sin(theta) * (burgerDiagonal / 2);
      }

      double translationY = 1.5 * line0HalfHeight + burger.getSpacing() - hOffset;

      if (!rightAngle)
      {
         angleDegrees = -angleDegrees;
         line1_translationX = -line1_translationX;
         translationX = -translationX;
      }

      return new KeyValue[] {new KeyValue(burger.rotateProperty(), 0, interpolation),
                             new KeyValue(line0.rotateProperty(), angleDegrees, interpolation),
                             new KeyValue(line0.translateXProperty(), translationX, interpolation),
                             new KeyValue(line0.translateYProperty(), translationY, interpolation),
                             new KeyValue(line0.scaleXProperty(), 0.5, interpolation),
                             new KeyValue(line0.opacityProperty(), 1, interpolation),
                             new KeyValue(line1.translateXProperty(), line1_translationX, interpolation),
                             new KeyValue(line1.translateYProperty(), 0, interpolation),
                             new KeyValue(line1.scaleXProperty(), 1, interpolation),
                             new KeyValue(line1.opacityProperty(), 0, interpolation),
                             new KeyValue(line2.rotateProperty(), -angleDegrees, interpolation),
                             new KeyValue(line2.translateXProperty(), translationX, interpolation),
                             new KeyValue(line2.translateYProperty(), -translationY, interpolation),
                             new KeyValue(line2.scaleXProperty(), 0.5, interpolation),
                             new KeyValue(line2.opacityProperty(), 1, interpolation)};
   }

   private KeyValue[] closeKeyValues(boolean closeRight)
   {
      double burgerWidth = line0.getLayoutBounds().getWidth();
      double burgerHeight = line2.getBoundsInParent().getMaxY() - line0.getBoundsInParent().getMinY();
      double burgerHalfHeight = burgerHeight / 2;

      double hypotenuse = EuclidCoreTools.norm(burgerHeight, burgerWidth);
      double angle = Math.toDegrees(Math.asin(burgerWidth / hypotenuse)) + 80;

      double line0_translationY = burgerHalfHeight - line0.getBoundsInLocal().getHeight() / 2;
      double line2_translationY = -burgerHalfHeight + line2.getBoundsInLocal().getHeight() / 2;
      double line1_translationX = -burger.getWidth() / 1.1;

      if (closeRight)
      {
         angle = -angle;
         line1_translationX = -line1_translationX;
      }

      return new KeyValue[] {new KeyValue(burger.rotateProperty(), 0, interpolation),
                             new KeyValue(line0.rotateProperty(), angle, interpolation),
                             new KeyValue(line0.translateXProperty(), 0, interpolation),
                             new KeyValue(line0.translateYProperty(), line0_translationY, interpolation),
                             new KeyValue(line0.scaleXProperty(), 1, interpolation),
                             new KeyValue(line2.rotateProperty(), -angle, interpolation),
                             new KeyValue(line2.translateXProperty(), 0, interpolation),
                             new KeyValue(line2.translateYProperty(), line2_translationY, interpolation),
                             new KeyValue(line2.scaleXProperty(), 1, interpolation),
                             new KeyValue(line1.opacityProperty(), 0, interpolation),
                             new KeyValue(line1.translateXProperty(), line1_translationX, interpolation),
                             new KeyValue(line1.translateYProperty(), 0, interpolation),
                             new KeyValue(line1.scaleXProperty(), 1, interpolation)};
   }

   public Transition getAnimation(JFXHamburger burger)
   {
      return new HamburgerAnimationTransition(burger);
   }
}
