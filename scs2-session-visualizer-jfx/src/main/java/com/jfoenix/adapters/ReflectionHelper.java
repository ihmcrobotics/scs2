package com.jfoenix.adapters;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

/**
 * This class is for breaking the module system of Java 9.
 *
 * @author huang
 */
public class ReflectionHelper
{

   @SuppressWarnings("unchecked")
   public static <T> T invoke(Class<?> cls, Object obj, String methodName)
   {
      try
      {
         Method method = cls.getDeclaredMethod(methodName);
         method.setAccessible(true);
         return (T) method.invoke(obj);
      }
      catch (Throwable ex)
      {
         throw new InternalError(ex);
      }
   }

   public static <T> T invoke(Object obj, String methodName)
   {
      return invoke(obj.getClass(), obj, methodName);
   }

   public static Method getMethod(Class<?> cls, String methodName)
   {
      try
      {
         Method method = cls.getDeclaredMethod(methodName);
         method.setAccessible(true);
         return method;
      }
      catch (Throwable ex)
      {
         throw new InternalError(ex);
      }
   }

   public static Field getField(Class<?> cls, String fieldName)
   {
      try
      {
         Field field = cls.getDeclaredField(fieldName);
         field.setAccessible(true);
         return field;
      }
      catch (Throwable ex)
      {
         return null;
      }
   }

   public static <T> T getFieldContent(Object obj, String fieldName)
   {
      return getFieldContent(obj.getClass(), obj, fieldName);
   }

   @SuppressWarnings("unchecked")
   public static <T> T getFieldContent(Class<?> cls, Object obj, String fieldName)
   {
      try
      {
         Field field = cls.getDeclaredField(fieldName);
         field.setAccessible(true);
         return (T) field.get(obj);
      }
      catch (Throwable ex)
      {
         return null;
      }
   }

   public static void setFieldContent(Class<?> cls, Object obj, String fieldName, Object content)
   {
      try
      {
         Field field = cls.getDeclaredField(fieldName);
         field.setAccessible(true);
         field.set(obj, content);
      }
      catch (Throwable ex)
      {
      }
   }
}
