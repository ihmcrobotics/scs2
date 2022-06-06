package us.ihmc.scs2.simulation;

import java.util.function.DoubleConsumer;

/**
 * Represents an operation that accepts time value and returns no value.
 * 
 * @author Sylvain Bertrand
 */
public interface TimeConsumer extends DoubleConsumer
{
   /**
    * Performs an operation given the current time value (expected to be in seconds).
    * 
    * @param time the current time value, typically expressed in seconds.
    */
   @Override
   void accept(double time);
}
