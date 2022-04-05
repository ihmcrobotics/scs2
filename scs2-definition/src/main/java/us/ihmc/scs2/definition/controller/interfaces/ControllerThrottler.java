package us.ihmc.scs2.definition.controller.interfaces;

import java.util.concurrent.TimeUnit;

import us.ihmc.commons.Conversions;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Wrapper around a {@link Controller} that allows to control the update rate.
 */
public class ControllerThrottler implements Controller
{
   private final String name;
   private final YoRegistry registry;

   private final Controller controller;
   private final YoDouble desiredControllerPeriod;
   private final YoDouble nextControllerDoControlTime;

   private DoubleProvider timeProvider;

   /**
    * Wraps the given controller.
    * <p>
    * The default behavior is to not throttle the controller, but this can be changed by setting the
    * period via {@link #setDesiredControllerPeriod(double)}.
    * </p>
    * 
    * @param controller the controller to wrap.
    * @see #setDesiredControllerPeriod(double)
    * @see #setDesiredControllerPeriod(long, TimeUnit)
    */
   public ControllerThrottler(Controller controller)
   {
      this.controller = controller;

      String controllerName = controller.getName();
      if (controllerName == null)
         controller.getClass().getSimpleName();

      YoRegistry controllerRegistry = controller.getYoRegistry();
      if (controllerRegistry == null)
         controllerRegistry = new YoRegistry(controllerName);

      name = controllerName;
      registry = controllerRegistry;

      desiredControllerPeriod = new YoDouble("desiredControllerPeriod" + name, registry);
      nextControllerDoControlTime = new YoDouble("nextControllerDoControlTime" + name, registry);
   }

   /**
    * Wraps the given controller and initialize the throttling period to use.
    * 
    * @param controller      the controller to wrap.
    * @param periodInSeconds the throttling period in seconds. Be mindful that this period should be a
    *                        multiple of the session period (e.g. simulation DT) or the update rate
    *                        will be inaccurate.
    */
   public ControllerThrottler(Controller controller, double periodInSeconds)
   {
      this(controller);
      setDesiredControllerPeriod(periodInSeconds);
   }

   /**
    * Wraps the given controller and initialize the throttling period to use.
    * 
    * @param controller the controller to wrap.
    * @param period     the throttling period. Be mindful that this period should be a multiple of the
    *                   session period (e.g. simulation DT) or the update rate will be inaccurate.
    * @param timeUnit   the unit of time for the period.
    */
   public ControllerThrottler(Controller controller, long period, TimeUnit timeUnit)
   {
      this(controller);
      setDesiredControllerPeriod(period, timeUnit);
   }

   /**
    * Sets the clock to use for measuring time elapsed between calls of {@link #doControl()}.
    * 
    * @param timeProvider the clock to use internally.
    */
   public void setTimeProvider(DoubleProvider timeProvider)
   {
      this.timeProvider = timeProvider;
   }

   /**
    * Sets the throttling period to use.
    * 
    * @param periodInSeconds the throttling period in seconds. Be mindful that this period should be a
    *                        multiple of the session period (e.g. simulation DT) or the update rate
    *                        will be inaccurate.
    */
   public void setDesiredControllerPeriod(double periodInSeconds)
   {
      if (periodInSeconds < 0.0)
         throw new IllegalArgumentException("The period cannot be negative: " + periodInSeconds);
      desiredControllerPeriod.set(periodInSeconds);
   }

   /**
    * Sets the throttling period to use.
    * 
    * @param period   the throttling period. Be mindful that this period should be a multiple of the
    *                 session period (e.g. simulation DT) or the update rate will be inaccurate.
    * @param timeUnit the unit of time for the period.
    */
   public void setDesiredControllerPeriod(long period, TimeUnit timeUnit)
   {
      if (period < 0)
         throw new IllegalArgumentException("The period cannot be negative: " + period);

      setDesiredControllerPeriod(Conversions.nanosecondsToSeconds(timeUnit.toNanos(period)));
   }

   @Override
   public void initialize()
   {
      nextControllerDoControlTime.set(timeProvider.getValue());
      controller.initialize();
   }

   @Override
   public void doControl()
   {
      if (timeProvider.getValue() < nextControllerDoControlTime.getValue())
         return;

      nextControllerDoControlTime.add(desiredControllerPeriod.getValue());
      controller.doControl();
   }

   @Override
   public void pause()
   {
      controller.pause();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }
}
