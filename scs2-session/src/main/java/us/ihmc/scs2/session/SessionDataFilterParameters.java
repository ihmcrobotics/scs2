package us.ihmc.scs2.session;

import java.util.function.Predicate;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class SessionDataFilterParameters
{
   /** This filter name. */
   private String name = null;
   /** Filter for selecting variables. */
   private Predicate<YoVariable> variableFilter = null;
   /** Filter for filtering registries. */
   private Predicate<YoRegistry> registryFilter = null;

   public SessionDataFilterParameters()
   {
   }

   public SessionDataFilterParameters(String name, Predicate<YoVariable> variableFilter, Predicate<YoRegistry> registryFilter)
   {
      this.name = name;
      this.variableFilter = variableFilter;
      this.registryFilter = registryFilter;
   }

   public SessionDataFilterParameters(SessionDataFilterParameters other)
   {
      this(other.name, other.variableFilter, other.registryFilter);
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setVariableFilter(Predicate<YoVariable> variableFilter)
   {
      this.variableFilter = variableFilter;
   }

   public void setRegistryFilter(Predicate<YoRegistry> registryFilter)
   {
      this.registryFilter = registryFilter;
   }

   public String getName()
   {
      return name;
   }

   public Predicate<YoVariable> getVariableFilter()
   {
      return variableFilter;
   }

   public Predicate<YoRegistry> getRegistryFilter()
   {
      return registryFilter;
   }

   @Override
   public String toString()
   {
      return "[name=" + name + ", variableFilter=" + variableFilter + ", registryFilter=" + registryFilter + "]";
   }
}
