package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Arrays;
import java.util.List;

import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "sdf")
public class SDFRoot implements SDFItem
{
   private String version;

   private SDFWorld world;

   private List<SDFModel> models;

   public String getVersion()
   {
      return version;
   }

   @XmlAttribute(name = "version")
   public void setVersion(String version)
   {
      this.version = version;
   }

   public SDFWorld getWorld()
   {
      return world;
   }

   @XmlElement(name = "world")
   public void setWorld(SDFWorld world)
   {
      this.world = world;
   }

   public List<SDFModel> getModels()
   {
      return models;
   }

   @XmlElement(name = "model")
   public void setModels(List<SDFModel> models)
   {
      this.models = models;
   }

   @Override
   public String getContentAsString()
   {
      return format("[version: %s, world: %s, models: %s]", version, world, models);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemListsURIHolders(Arrays.asList(world), models);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }
}
