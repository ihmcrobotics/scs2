package us.ihmc.scs2.definition.yoComposite;

import us.ihmc.scs2.definition.yoChart.YoChartGroupModelDefinition;

import javax.xml.bind.annotation.*;
import java.util.*;

/**
 * Definition for a composite pattern. A composite pattern is used to group multiple {@link us.ihmc.yoVariables.variable.YoVariable}s together to represent a
 * single a complex data type such as a quaternion or a vector.
 */
@XmlRootElement(name = "YoCompositePattern")
@XmlType(propOrder = {"name", "crossRegistry", "ids", "altIds", "preferredConfigurations"})
public class YoCompositePatternDefinition
{
   /**
    * The name of the composite pattern.
    */
   private String name;
   /**
    * Whether the components of this composite pattern are expected to be declared across multiple registries.
    */
   private boolean crossRegistry = false;
   /**
    * The identifiers of the components of this composite pattern.
    */
   private String[] identifiers;
   /**
    * The alternate identifiers of the components of this composite pattern.
    * <p>
    * Mostly used to allow flexibility in the naming of the components of the composite pattern. For instance, a quaternion can be defined as a composite
    * pattern with the identifiers {@code x,y,z,s} or {@code x,y,z,w}.
    * </p>
    */
   private final List<String> altIds = new ArrayList<>();
   /**
    * The preferred configurations of the composite pattern.
    * <p>
    * A configuration is used to define chart layouts for the components of the composite.
    * </p>
    */
   private final List<YoChartGroupModelDefinition> preferredConfigurations = new ArrayList<>();

   /**
    * Creates an empty composite pattern definition. Typically used for XML marshalling.
    */
   public YoCompositePatternDefinition()
   {
   }

   /**
    * Creates and initializes a composite pattern definition.
    *
    * @param name the name of this composite pattern.
    */
   public YoCompositePatternDefinition(String name)
   {
      setName(name);
   }

   /**
    * Copy constructor.
    *
    * @param other the other composite pattern definition to copy. Not modified.
    */
   public YoCompositePatternDefinition(YoCompositePatternDefinition other)
   {
      if (other == null)
         return;

      setName(other.name);
      setCrossRegistry(other.crossRegistry);
      identifiers = other.identifiers;
      altIds.addAll(other.altIds);
      setPreferredConfigurations(other.preferredConfigurations);
   }

   /**
    * Sets the name of this composite pattern.
    * <p>
    * The name is typically used to identify the type this composite pattern represents. For instance "Quaternion" or "Tuple3D".
    * </p>
    *
    * @param name the name of this composite pattern.
    */
   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   /**
    * Sets whether the components of this composite pattern are expected to be declared across multiple registries.
    * <p>
    * This is typically the case for composite patterns that represent a complex data type such as a quaternion or a vector.
    * </p>
    *
    * @param crossRegistry whether the components of this composite pattern are expected to be declared across multiple registries.
    */
   @XmlAttribute
   public void setCrossRegistry(boolean crossRegistry)
   {
      this.crossRegistry = crossRegistry;
   }

   /**
    * Sets the identifiers of the components of this composite pattern.
    * <p>
    * The identifiers are used to identify the components of the composite pattern. For instance, a quaternion can be defined as a composite pattern with the
    * identifiers {@code x,y,z,s} or {@code x,y,z,w}.
    * </p>
    *
    * @param identifiers the identifiers of the components of this composite pattern.
    */
   public void setIdentifiers(String[] identifiers)
   {
      this.identifiers = identifiers;
   }

   /**
    * Used for XML marshalling. Use {@link #setIdentifiers(String[])} instead.
    *
    * @param ids the identifiers of the components of this composite pattern stored in a CSV format.
    */
   @XmlAttribute
   public void setIds(String ids)
   {
      this.identifiers = ids.replaceAll(" ", "").split(",");
   }

   /**
    * Used for XML marshalling.
    *
    * @param altIds the alternate identifiers of the components of this composite pattern stored in a CSV format.
    */
   @XmlElement
   public void setAltIds(List<String> altIds)
   {
      this.altIds.clear();
      this.altIds.addAll(altIds);
   }

   /**
    * Sets the preferred configurations of the composite pattern.
    * <p>
    * A configuration is used to define chart layouts for the components of the composite.
    * </p>
    *
    * @param preferredConfigurations the preferred configurations of the composite pattern.
    */
   @XmlElement
   public void setPreferredConfigurations(List<YoChartGroupModelDefinition> preferredConfigurations)
   {
      this.preferredConfigurations.clear();
      for (YoChartGroupModelDefinition model : preferredConfigurations)
         this.preferredConfigurations.add(model.clone());
   }

   /**
    * Returns the name of this composite pattern.
    * <p>
    * The name is typically used to identify the type this composite pattern represents. For instance "Quaternion" or "Tuple3D".
    * </p>
    *
    * @return the name of this composite pattern.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Returns whether the components of this composite pattern are expected to be declared across multiple registries.
    * <p>
    * This is typically the case for composite patterns that represent a complex data type such as a quaternion or a vector.
    * </p>
    *
    * @return whether the components of this composite pattern are expected to be declared across multiple registries.
    */
   public boolean isCrossRegistry()
   {
      return crossRegistry;
   }

   /**
    * Returns the identifiers of the components of this composite pattern.
    * <p>
    * The identifiers are used to identify the components of the composite pattern. For instance, a quaternion can be defined as a composite pattern with the
    * identifiers {@code x,y,z,s} or {@code x,y,z,w}.
    * </p>
    *
    * @return the identifiers of the components of this composite pattern.
    */
   @XmlTransient
   public String[] getIdentifiers()
   {
      return identifiers;
   }

   /**
    * Only used for XML marshalling. Use {@link #getAlternateIdentifiers()} instead.
    *
    * @return the list of alternate identifiers.
    */
   public List<String> getAltIds()
   {
      return altIds;
   }

   /**
    * Returns the alternate identifiers of the components of this composite pattern.
    * <p>
    * Mostly used to allow flexibility in the naming of the components of the composite pattern. For instance, a quaternion can be defined as a composite
    * pattern with the identifiers {@code x,y,z,s} or {@code x,y,z,w}.
    * </p>
    *
    * @return the alternate identifiers of the components of this composite pattern.
    */
   @XmlTransient
   public List<String[]> getAlternateIdentifiers()
   {
      if (altIds.isEmpty())
         return Collections.emptyList();
      return altIds.stream().map(ids -> ids.split(",")).toList();
   }

   /**
    * Returns the preferred configurations of the composite pattern.
    * <p>
    * A configuration is used to define chart layouts for the components of the composite.
    * </p>
    *
    * @return the preferred configurations of the composite pattern.
    */
   public List<YoChartGroupModelDefinition> getPreferredConfigurations()
   {
      return preferredConfigurations;
   }

   /**
    * Returns a deep copy of this composite pattern definition.
    *
    * @return a deep copy of this composite pattern definition.
    */
   @Override
   public YoCompositePatternDefinition clone()
   {
      return new YoCompositePatternDefinition(this);
   }

   /**
    * Tests on a per-field basis if this composite pattern definition is equal to the given {@code object}.
    *
    * @param object the other object to compare against this.
    * @return {@code true} if the two composite pattern definitions are equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoCompositePatternDefinition other)
      {
         if (!Objects.equals(name, other.name))
            return false;
         if (crossRegistry != other.crossRegistry)
            return false;
         if (!Arrays.equals(identifiers, other.identifiers))
            return false;
         if (!Objects.equals(altIds, other.altIds))
            return false;
         if (!Objects.equals(preferredConfigurations, other.preferredConfigurations))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this composite pattern definition as follows:
    * <pre>
    *    name: name, ids: [id1, id2, ..., idn], chart ids: [chartId1, chartId2, ..., chartIdn]
    * </pre>
    *
    * @return a {@code String} representation of this composite pattern definition.
    */
   @Override
   public String toString()
   {
      return "name: " + name + ", ids: " + Arrays.toString(identifiers) + ", chart ids: " + preferredConfigurations;
   }
}
