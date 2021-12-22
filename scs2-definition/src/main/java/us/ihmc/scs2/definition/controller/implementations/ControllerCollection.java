package us.ihmc.scs2.definition.controller.implementations;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ControllerCollection implements Controller
{
   private final YoRegistry registry;
   private final List<Controller> controllers = new ArrayList<>();

   public ControllerCollection(String name)
   {
      this(name, Collections.emptyList());
   }

   public ControllerCollection(String name, Controller... controllers)
   {
      this(name, Arrays.asList(controllers));
   }

   public ControllerCollection(String name, Iterable<? extends Controller> controllers)
   {
      registry = new YoRegistry(name);
      addControllers(controllers);
   }

   public void addController(Controller controller)
   {
      if (controller.getYoRegistry() != null)
         registry.addChild(controller.getYoRegistry());
      controllers.add(controller);
   }

   public void addControllers(Controller... controllers)
   {
      for (Controller controller : controllers)
      {
         addController(controller);
      }
   }

   public void addControllers(Iterable<? extends Controller> controllers)
   {
      for (Controller controller : controllers)
      {
         addController(controller);
      }
   }

   public boolean removeController(Controller controller)
   {
      return controllers.remove(controller);
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < controllers.size(); i++)
         controllers.get(i).initialize();
   }

   @Override
   public void doControl()
   {
      for (int i = 0; i < controllers.size(); i++)
         controllers.get(i).doControl();
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
