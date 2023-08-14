package us.ihmc.scs2.sessionVisualizer.sliderboard;

public interface MidiChannelConfig
{
   public int getChannel();

   default int getMin()
   {
      throw new RuntimeException("No min implemented");
   }

   default int getMax()
   {
      throw new RuntimeException("No max implemented");
   }

   public int ordinal();
}
