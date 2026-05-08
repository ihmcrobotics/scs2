package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import static org.junit.jupiter.api.Assertions.assertEquals;

import javafx.beans.property.SimpleLongProperty;
import org.junit.jupiter.api.Test;
import us.ihmc.scs2.sessionVisualizer.jfx.session.log.LogSessionManagerController.TimeStringBinding;

public class LogSessionManagerControllerTest
{
   @Test
   public void testTimeStringBindingClampsNegativeTimeToZero()
   {
      SimpleLongProperty logTimeNanos = new SimpleLongProperty();
      TimeStringBinding timeStringBinding = new TimeStringBinding(logTimeNanos, Number::longValue);

      logTimeNanos.set(-1_000_000);

      assertEquals("00s000", timeStringBinding.get());
   }

   @Test
   public void testTimeStringBindingFormatsPositiveTime()
   {
      SimpleLongProperty logTimeNanos = new SimpleLongProperty();
      TimeStringBinding timeStringBinding = new TimeStringBinding(logTimeNanos, Number::longValue);

      logTimeNanos.set(1_234_000_000);

      assertEquals("01s234", timeStringBinding.get());
   }
}
