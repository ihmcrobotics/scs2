package us.ihmc.scs2.simulation;

import java.util.function.DoubleConsumer;

public interface TimeConsumer extends DoubleConsumer
{
   @Override
   void accept(double time);
}
