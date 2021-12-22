package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;

public class SDFWorld implements SDFItem
{
   private List<SDFModel> models;
   private List<Road> roads;

   public List<SDFModel> getModels()
   {
      return models;
   }

   @XmlElement(name = "model")
   public void setModels(List<SDFModel> models)
   {
      this.models = models;
   }

   public List<Road> getRoads()
   {
      return roads;
   }

   @XmlElement(name = "road")
   public void setRoads(List<Road> roads)
   {
      this.roads = roads;
   }

   @Override
   public String getContentAsString()
   {
      return format("[models: %s, roads: %s]", models, roads);
   }

   @Override
   public List<SDFURIHolder> getURIHolders()
   {
      return SDFItem.combineItemListsURIHolders(models, roads);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   public static class Road implements SDFItem
   {
      private String name;
      private String width;
      private List<String> points;

      public String getName()
      {
         return name;
      }

      @XmlAttribute(name = "name")
      public void setName(String name)
      {
         this.name = name;
      }

      public String getWidth()
      {
         return width;
      }

      @XmlElement(name = "width")
      public void setWidth(String width)
      {
         this.width = width;
      }

      public List<String> getPoints()
      {
         return points;
      }

      @XmlElement(name = "point")
      public void setPoints(List<String> points)
      {
         this.points = points;
      }

      @Override
      public String getContentAsString()
      {
         return format("[name: %s, width: %s, points: %s]", name, width, points);
      }

      @Override
      public List<SDFURIHolder> getURIHolders()
      {
         return Collections.emptyList();
      }

      @Override
      public String toString()
      {
         return itemToString();
      }
   }
}