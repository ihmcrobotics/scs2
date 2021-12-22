package us.ihmc.scs2.definition.visual;

import java.util.ArrayList;
import java.util.List;

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
}
