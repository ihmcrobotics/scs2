package us.ihmc.scs2.definition.visual;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class MaterialScriptDefinition
{
   private String name;
   private List<String> urls;
   private List<String> resourceDirectories;

   public MaterialScriptDefinition()
   {
   }

   public MaterialScriptDefinition(MaterialScriptDefinition other)
   {
      name = other.name;
      urls = other.urls == null ? null : new ArrayList<>(other.urls);
      resourceDirectories = other.resourceDirectories == null ? null : new ArrayList<>(other.resourceDirectories);
   }

   public String getName()
   {
      return name;
   }

   public List<String> getUrls()
   {
      return urls;
   }

   public List<String> getResourceDirectories()
   {
      return resourceDirectories;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setUrls(List<String> urls)
   {
      this.urls = urls;
   }

   public void setResourceDirectories(List<String> resourceDirectories)
   {
      this.resourceDirectories = resourceDirectories;
   }

   public MaterialScriptDefinition copy()
   {
      return new MaterialScriptDefinition(this);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, name);
      bits = EuclidHashCodeTools.addToHashCode(bits, urls);
      bits = EuclidHashCodeTools.addToHashCode(bits, resourceDirectories);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      if (object == null)
         return false;
      if (getClass() != object.getClass())
         return false;

      MaterialScriptDefinition other = (MaterialScriptDefinition) object;

      if (!Objects.equals(name, other.name))
         return false;
      if (!Objects.equals(urls, other.urls))
         return false;
      if (!Objects.equals(resourceDirectories, other.resourceDirectories))
         return false;

      return true;
   }
}
