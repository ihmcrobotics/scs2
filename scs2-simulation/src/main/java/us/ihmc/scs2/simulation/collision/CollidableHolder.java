package us.ihmc.scs2.simulation.collision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public interface CollidableHolder
{
   default int getNumberOfCollidables()
   {
      return getCollidables().size();
   }

   List<Collidable> getCollidables();

   public static CollidableHolder fromCollection(Collection<Collidable> collidables)
   {
      List<Collidable> collidableList = new ArrayList<>(collidables);
      return () -> collidableList;
   }
}