package us.ihmc.scs2.definition.terrain;

import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlTransient;

import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

@XmlRootElement(name = "Terrain")
public class TerrainObjectDefinition
{
   private String name;
   private List<VisualDefinition> visualDefinitions = new ArrayList<>();
   private List<CollisionShapeDefinition> collisionShapeDefinitions = new ArrayList<>();

   private ClassLoader resourceClassLoader;

   public TerrainObjectDefinition()
   {
   }

   public TerrainObjectDefinition(VisualDefinition visualDefinition, CollisionShapeDefinition collisionShapeDefinition)
   {
      visualDefinitions.add(visualDefinition);
      collisionShapeDefinitions.add(collisionShapeDefinition);
   }

   public TerrainObjectDefinition(TerrainObjectDefinition other)
   {
      name = other.name;
      for (VisualDefinition visualDefinition : other.visualDefinitions)
         visualDefinitions.add(visualDefinition.copy());
      for (CollisionShapeDefinition collisionShapeDefinition : other.collisionShapeDefinitions)
         collisionShapeDefinitions.add(collisionShapeDefinition.copy());
   }

   @XmlElement
   public void setName(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   @XmlElement(name = "visual")
   public void setVisualDefinitions(List<VisualDefinition> visualDefinitions)
   {
      this.visualDefinitions = visualDefinitions;
   }

   public void addVisualDefinition(VisualDefinition visualDefinition)
   {
      visualDefinitions.add(visualDefinition);
   }

   public List<VisualDefinition> getVisualDefinitions()
   {
      return visualDefinitions;
   }

   @XmlElement(name = "collision")
   public void setCollisionShapeDefinitions(List<CollisionShapeDefinition> collisionShapeDefinitions)
   {
      this.collisionShapeDefinitions = collisionShapeDefinitions;
   }

   public void addCollisionShapeDefinition(CollisionShapeDefinition collisionShapeDefinition)
   {
      collisionShapeDefinitions.add(collisionShapeDefinition);
   }

   public List<CollisionShapeDefinition> getCollisionShapeDefinitions()
   {
      return collisionShapeDefinitions;
   }

   @XmlTransient
   public void setResourceClassLoader(ClassLoader resourceClassLoader)
   {
      this.resourceClassLoader = resourceClassLoader;
   }

   public ClassLoader getResourceClassLoader()
   {
      return resourceClassLoader;
   }

   public TerrainObjectDefinition copy()
   {
      return new TerrainObjectDefinition(this);
   }
}
