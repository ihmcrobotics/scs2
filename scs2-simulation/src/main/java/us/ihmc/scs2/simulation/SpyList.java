package us.ihmc.scs2.simulation;

import java.util.AbstractList;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.ListIterator;
import java.util.Objects;

public class SpyList<E> extends AbstractList<E>
{
   private final List<E> list;
   private final List<ChangeListener<E>> listeners = new ArrayList<>();

   public SpyList()
   {
      list = new ArrayList<>();
   }

   public SpyList(List<E> list)
   {
      this.list = list;
   }

   public void addListener(ChangeListener<E> listener)
   {
      listeners.add(listener);
   }

   public boolean removeListener(ChangeListener<E> listener)
   {
      return listeners.remove(listener);
   }

   public void removeAllListeners()
   {
      listeners.clear();
   }

   @Override
   public E get(int index)
   {
      return list.get(index);
   }

   @Override
   public E set(int index, E element)
   {
      E oldElement = list.set(index, element);
      notifyChange(ChangeImpl.replaced(index, element, oldElement));
      return oldElement;
   }

   @Override
   public void add(int index, E element)
   {
      list.add(index, element);
      notifyChange(ChangeImpl.added(index, element));
   }

   @Override
   public E remove(int index)
   {
      E oldElement = list.remove(index);
      notifyChange(ChangeImpl.removed(index, oldElement));
      return oldElement;
   }

   @Override
   public void clear()
   {
      if (list.isEmpty())
         return;

      List<E> oldElements = new ArrayList<>(list);
      list.clear();
      notifyChange(ChangeImpl.removed(0, oldElements));
   }

   @Override
   public boolean addAll(Collection<? extends E> c)
   {
      return addAll(size(), c);
   }

   @Override
   public boolean addAll(int index, Collection<? extends E> c)
   {
      if (list.addAll(index, c))
      {
         notifyChange(ChangeImpl.added(index, new ArrayList<>(c)));
         return true;
      }
      return false;
   }

   @Override
   protected void removeRange(int fromIndex, int toIndex)
   {
      ListIterator<E> it = list.listIterator(fromIndex);
      List<E> oldElements = new ArrayList<>();

      for (int i = 0, n = toIndex - fromIndex; i < n; i++)
      {
         oldElements.add(it.next());
         it.remove();
      }

      notifyChange(ChangeImpl.removed(fromIndex, oldElements));
   }

   @Override
   public int size()
   {
      return list.size();
   }

   private void notifyChange(Change<E> change)
   {
      for (ChangeListener<E> listener : listeners)
      {
         listener.onChange(change);
      }
   }

   public interface ChangeListener<E>
   {
      void onChange(Change<E> change);
   }

   public interface Change<E>
   {
      boolean wasAdded();

      boolean wasRemoved();

      boolean wasReplaced();

      int getIndex();

      int getSize();

      List<E> getNewElements();

      List<E> getOldElements();
   }

   private static class ChangeImpl<E> implements Change<E>
   {
      private final boolean wasAdded;
      private final boolean wasRemoved;
      private final boolean wasReplaced;
      private final int index;
      private final int size;
      private final List<E> newElements;
      private final List<E> oldElements;

      public static <E> Change<E> added(int index, E newElement)
      {
         return new ChangeImpl<>(true, false, false, index, newElement, null);
      }

      public static <E> Change<E> added(int index, List<E> newElements)
      {
         return new ChangeImpl<>(true, false, false, index, newElements.size(), new ArrayList<>(newElements), null);
      }

      public static <E> Change<E> removed(int index, E oldElement)
      {
         return new ChangeImpl<>(false, true, false, index, null, oldElement);
      }

      public static <E> Change<E> removed(int index, List<E> oldElements)
      {
         return new ChangeImpl<>(false, true, false, index, oldElements.size(), null, new ArrayList<>(oldElements));
      }

      public static <E> Change<E> replaced(int index, E newElement, E oldElement)
      {
         return new ChangeImpl<>(false, false, true, index, newElement, oldElement);
      }

      public ChangeImpl(boolean wasAdded, boolean wasRemoved, boolean wasReplaced, int index, E newElement, E oldElement)
      {
         this(wasAdded, wasRemoved, wasReplaced, index, 1, newElement == null ? null : Collections.singletonList(newElement),
              oldElement == null ? null : Collections.singletonList(oldElement));
      }

      public ChangeImpl(boolean wasAdded, boolean wasRemoved, boolean wasReplaced, int index, int size, List<E> newElements, List<E> oldElements)
      {
         this.wasAdded = wasAdded;
         this.wasRemoved = wasRemoved;
         this.wasReplaced = wasReplaced;
         this.index = index;
         this.size = size;
         this.newElements = newElements;
         this.oldElements = oldElements;
      }

      @Override
      public boolean wasAdded()
      {
         return wasAdded;
      }

      @Override
      public boolean wasRemoved()
      {
         return wasRemoved;
      }

      @Override
      public boolean wasReplaced()
      {
         return wasReplaced;
      }

      @Override
      public int getIndex()
      {
         return index;
      }

      @Override
      public int getSize()
      {
         return size;
      }

      @Override
      public List<E> getNewElements()
      {
         return newElements;
      }

      @Override
      public List<E> getOldElements()
      {
         return oldElements;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof Change)
         {
            Change<?> other = (Change<?>) object;
            if (wasAdded != other.wasAdded())
               return false;
            if (wasRemoved != other.wasRemoved())
               return false;
            if (wasReplaced != other.wasReplaced())
               return false;
            if (index != other.getIndex())
               return false;
            if (size != other.getSize())
               return false;
            if (!Objects.equals(newElements, other.getNewElements()))
               return false;
            if (!Objects.equals(oldElements, other.getOldElements()))
               return false;
            return true;
         }
         else
         {
            return false;
         }
      }

      @Override
      public int hashCode()
      {
         return Objects.hash(wasAdded, wasRemoved, wasReplaced, newElements, oldElements);
      }
   }
}
