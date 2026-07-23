package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;

/** Minimal, JavaFX-free {@link YoBufferPropertiesReadOnly} stub for constructing test {@code BufferSample}s. */
class TestBufferProperties implements YoBufferPropertiesReadOnly
{
   private final int size, currentIndex, inPoint, outPoint;

   TestBufferProperties(int size, int currentIndex, int inPoint, int outPoint)
   {
      this.size = size;
      this.currentIndex = currentIndex;
      this.inPoint = inPoint;
      this.outPoint = outPoint;
   }

   @Override
   public int getSize()
   {
      return size;
   }

   @Override
   public int getCurrentIndex()
   {
      return currentIndex;
   }

   @Override
   public int getInPoint()
   {
      return inPoint;
   }

   @Override
   public int getOutPoint()
   {
      return outPoint;
   }
}
