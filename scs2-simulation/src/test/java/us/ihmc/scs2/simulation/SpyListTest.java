package us.ihmc.scs2.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.simulation.SpyList.Change;

public class SpyListTest
{

   private static final int ITERATIONS = 1000;

   @Test
   public void testSet()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      { // set(int index, E element)
         int size = random.nextInt(100);
         SpyList<Long> spyList = nextSpyList(random, size);

         List<Change<Long>> changes = new ArrayList<>();
         spyList.addListener(changes::add);

         Long newElement = random.nextLong();

         if (size == 0)
         {
            assertThrows(IndexOutOfBoundsException.class, () -> spyList.set(0, newElement));
         }
         else
         {
            int index = random.nextInt(size);
            Long oldElement = spyList.set(index, newElement);

            assertEquals(1, changes.size());
            Change<Long> change = changes.get(0);
            assertEquals(false, change.wasAdded());
            assertEquals(false, change.wasRemoved());
            assertEquals(true, change.wasReplaced());
            assertEquals(index, change.getIndex());
            assertEquals(1, change.getSize());
            assertEquals(1, change.getNewElements().size());
            assertTrue(newElement == change.getNewElements().get(0));
            assertEquals(1, change.getOldElements().size());
            assertTrue(oldElement == change.getOldElements().get(0));
         }
      }
   }

   @Test
   public void testAdd()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      { // add(E element)
         int size = random.nextInt(100);
         SpyList<Long> spyList = nextSpyList(random, size);

         List<Change<Long>> changes = new ArrayList<>();
         spyList.addListener(changes::add);

         Long newElement = random.nextLong();

         spyList.add(newElement);

         assertEquals(1, changes.size());
         Change<Long> change = changes.get(0);
         assertEquals(true, change.wasAdded());
         assertEquals(false, change.wasRemoved());
         assertEquals(false, change.wasReplaced());
         assertEquals(spyList.size() - 1, change.getIndex());
         assertEquals(1, change.getSize());
         assertEquals(1, change.getNewElements().size());
         assertTrue(newElement == change.getNewElements().get(0));
         assertNull(change.getOldElements());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // add(int index, E element)
         int size = random.nextInt(100);
         SpyList<Long> spyList = nextSpyList(random, size);

         List<Change<Long>> changes = new ArrayList<>();
         spyList.addListener(changes::add);

         int index = random.nextInt(size + 1);
         Long newElement = random.nextLong();

         spyList.add(index, newElement);

         assertEquals(1, changes.size());
         Change<Long> change = changes.get(0);
         assertEquals(true, change.wasAdded());
         assertEquals(false, change.wasRemoved());
         assertEquals(false, change.wasReplaced());
         assertEquals(index, change.getIndex());
         assertEquals(1, change.getSize());
         assertEquals(1, change.getNewElements().size());
         assertTrue(newElement == change.getNewElements().get(0));
         assertNull(change.getOldElements());
      }
   }

   @Test
   public void testRemoved()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      { // remove(int index)
         int size = random.nextInt(100);
         SpyList<Long> spyList = nextSpyList(random, size);

         List<Change<Long>> changes = new ArrayList<>();
         spyList.addListener(changes::add);

         if (size == 0)
         {
            assertThrows(IndexOutOfBoundsException.class, () -> spyList.remove(0));
         }
         else
         {
            int index = random.nextInt(size);
            Long oldElement = spyList.remove(index);

            assertEquals(1, changes.size());
            Change<Long> change = changes.get(0);
            assertEquals(false, change.wasAdded());
            assertEquals(true, change.wasRemoved());
            assertEquals(false, change.wasReplaced());
            assertEquals(index, change.getIndex());
            assertEquals(1, change.getSize());
            assertNull(change.getNewElements());
            assertEquals(1, change.getOldElements().size());
            assertTrue(oldElement == change.getOldElements().get(0));
         }
      }
   }

   @Test
   public void testClear()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      { // clear()
         int size = random.nextInt(100);
         SpyList<Long> spyList = nextSpyList(random, size);

         List<Change<Long>> changes = new ArrayList<>();
         spyList.addListener(changes::add);

         ArrayList<Long> oldElements = new ArrayList<>(spyList);

         spyList.clear();

         if (size == 0)
         {
            assertEquals(0, changes.size());
         }
         else
         {
            assertEquals(1, changes.size());
            Change<Long> change = changes.get(0);
            assertEquals(false, change.wasAdded());
            assertEquals(true, change.wasRemoved());
            assertEquals(false, change.wasReplaced());
            assertEquals(0, change.getIndex());
            assertEquals(oldElements.size(), change.getSize());
            assertNull(change.getNewElements());
            assertEquals(oldElements, change.getOldElements());
         }
      }
   }

   @Test
   public void testAddAll()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      { // addAll(Collection<? extends E> c)
         int size = random.nextInt(100);
         SpyList<Long> spyList = nextSpyList(random, size);

         List<Change<Long>> changes = new ArrayList<>();
         spyList.addListener(changes::add);

         List<Long> newElements = nextList(random, random.nextInt(100));

         spyList.addAll(newElements);

         if (newElements.isEmpty())
         {
            assertEquals(0, changes.size());
         }
         else
         {
            assertEquals(1, changes.size());
            Change<Long> change = changes.get(0);
            assertEquals(true, change.wasAdded());
            assertEquals(false, change.wasRemoved());
            assertEquals(false, change.wasReplaced());
            assertEquals(spyList.size() - newElements.size(), change.getIndex());
            assertEquals(newElements.size(), change.getSize());
            assertEquals(newElements, change.getNewElements());
            assertNull(change.getOldElements());
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // addAll(int index, Collection<? extends E> c)
         int size = random.nextInt(100);
         SpyList<Long> spyList = nextSpyList(random, size);

         List<Change<Long>> changes = new ArrayList<>();
         spyList.addListener(changes::add);

         int index = random.nextInt(size + 1);
         List<Long> newElements = nextList(random, random.nextInt(100));

         spyList.addAll(index, newElements);

         if (newElements.isEmpty())
         {
            assertEquals(0, changes.size());
         }
         else
         {
            assertEquals(1, changes.size());
            Change<Long> change = changes.get(0);
            assertEquals(true, change.wasAdded());
            assertEquals(false, change.wasRemoved());
            assertEquals(false, change.wasReplaced());
            assertEquals(index, change.getIndex());
            assertEquals(newElements.size(), change.getSize());
            assertEquals(newElements, change.getNewElements());
            assertNull(change.getOldElements());
         }
      }
   }

   public static SpyList<Long> nextSpyList(Random random, int size)
   {
      SpyList<Long> spyList = new SpyList<>();
      while (spyList.size() < size)
         spyList.add(random.nextLong());
      return spyList;
   }

   public static List<Long> nextList(Random random, int size)
   {
      List<Long> list = new ArrayList<>();
      while (list.size() < size)
         list.add(random.nextLong());
      return list;
   }
}
