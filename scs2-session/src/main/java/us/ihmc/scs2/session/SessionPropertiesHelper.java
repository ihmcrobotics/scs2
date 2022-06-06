package us.ihmc.scs2.session;

import java.security.AccessController;
import java.security.PrivilegedAction;
import java.util.Properties;

import us.ihmc.log.LogTools;

public class SessionPropertiesHelper
{
   @SuppressWarnings({"rawtypes", "unchecked"})
   private static final Properties systemProperties = (Properties) AccessController.doPrivileged((PrivilegedAction) () -> System.getProperties());;

   public static boolean loadBooleanProperty(String key, boolean defaultValue)
   {
      return loadBooleanProperty(systemProperties, key, defaultValue);
   }

   public static boolean loadBooleanProperty(Properties properties, String key, boolean defaultValue)
   {
      String stringValue = properties.getProperty(key);
      if (stringValue != null)
         return Boolean.parseBoolean(stringValue);
      else
         return defaultValue;
   }

   public static boolean loadBooleanProperty(String key, boolean defaultValue, boolean definedNotSetValue)
   {
      return loadBooleanProperty(systemProperties, key, defaultValue);
   }

   public static boolean loadBooleanProperty(Properties properties, String key, boolean defaultValue, boolean definedNotSetValue)
   {
      String stringValue = properties.getProperty(key);
      if (stringValue != null && stringValue.isEmpty())
         return definedNotSetValue;
      if (stringValue != null)
         return Boolean.parseBoolean(stringValue);
      else
         return defaultValue;
   }

   public static double loadDoubleProperty(String key, double defaultValue)
   {
      return loadDoubleProperty(systemProperties, key, defaultValue);
   }

   public static double loadDoubleProperty(Properties properties, String key, double defaultValue)
   {
      String stringValue = properties.getProperty(key);

      if (stringValue != null)
      {
         try
         {
            return Double.parseDouble(stringValue);
         }
         catch (NumberFormatException e)
         {
            LogTools.error("Exception while loading property {}: {}. Using default value.", key, e.getMessage());
         }
      }

      return defaultValue;
   }

   public static int loadIntegerProperty(String key, int defaultValue)
   {
      return loadIntegerProperty(systemProperties, key, defaultValue);
   }

   public static int loadIntegerProperty(Properties properties, String key, int defaultValue)
   {
      String stringValue = properties.getProperty(key);

      if (stringValue != null)
      {
         try
         {
            return Integer.parseInt(stringValue);
         }
         catch (NumberFormatException e)
         {
            LogTools.error("Exception while loading property {}: {}. Using default value.", key, e.getMessage());
         }
      }

      return defaultValue;
   }

   public static long loadLongProperty(String key, long defaultValue)
   {
      return loadLongProperty(systemProperties, key, defaultValue);
   }

   public static long loadLongProperty(Properties properties, String key, long defaultValue)
   {
      String stringValue = properties.getProperty(key);

      if (stringValue != null)
      {
         try
         {
            return Long.parseLong(stringValue);
         }
         catch (NumberFormatException e)
         {
            LogTools.error("Exception while loading property {}: {}. Using default value.", key, e.getMessage());
         }
      }

      return defaultValue;
   }
}
