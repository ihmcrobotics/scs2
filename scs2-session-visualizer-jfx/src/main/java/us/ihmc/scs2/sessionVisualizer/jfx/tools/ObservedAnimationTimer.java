package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import javafx.animation.AnimationTimer;

public abstract class ObservedAnimationTimer extends AnimationTimer
{
   private final String name;
   private long lastHandleDuration;

   public ObservedAnimationTimer()
   {
      this.name = getClass().getSimpleName();
   }

   public ObservedAnimationTimer(String name)
   {
      this.name = name;
   }

   @Override
   public final void handle(long now)
   {
      long start = System.nanoTime();
      handleImpl(now);
      long end = System.nanoTime();
      lastHandleDuration = end - start;
   }

   public abstract void handleImpl(long now);

   public String getName()
   {
      return name;
   }

   public long getLastHandleDuration()
   {
      return lastHandleDuration;
   }
}
