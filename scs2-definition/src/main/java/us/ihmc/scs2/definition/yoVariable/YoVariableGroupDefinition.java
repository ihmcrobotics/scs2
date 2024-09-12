package us.ihmc.scs2.definition.yoVariable;

import java.util.ArrayList;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlRootElement
public class YoVariableGroupDefinition
{
   private String name;
   private List<String> variableNames;
   private List<String> registryNames;

   public YoVariableGroupDefinition()
   {
   }

   @XmlAttribute
   public void setName(String name)
   {
      this.name = name;
   }

   public void addVariableName(String variableName)
   {
      if (variableNames == null)
         variableNames = new ArrayList<>();
      variableNames.add(variableName);
   }

   @XmlElement
   public void setVariableNames(List<String> variableNames)
   {
      this.variableNames = variableNames;
   }

   public void addRegistryName(String registryName)
   {
      if (registryNames == null)
         registryNames = new ArrayList<>();
      registryNames.add(registryName);
   }

   @XmlElement
   public void setRegistryNames(List<String> registryNames)
   {
      this.registryNames = registryNames;
   }

   public String getName()
   {
      return name;
   }

   public List<String> getVariableNames()
   {
      return variableNames;
   }

   public List<String> getRegistryNames()
   {
      return registryNames;
   }

   @Override
   public String toString()
   {
      return "[name=" + name + ", variableNames=" + variableNames + ", registryNames=" + registryNames + "]";
   }

}
