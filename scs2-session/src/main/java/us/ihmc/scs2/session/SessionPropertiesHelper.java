package us.ihmc.scs2.session;

import us.ihmc.log.LogTools;

public class SessionPropertiesHelper
{
   public static boolean loadBooleanProperty(String propertyKey, boolean defaultValue)
   {
      String stringValue = System.getProperty(propertyKey);
      if (stringValue != null)
         return Boolean.parseBoolean(stringValue);
      else
         return defaultValue;
   }

   public static boolean loadBooleanProperty(String propertyKey, boolean defaultValue, boolean definedNotSetValue)
   {
      String stringValue = System.getProperty(propertyKey);
      if (stringValue != null && stringValue.isEmpty())
         return definedNotSetValue;
      if (stringValue != null)
         return Boolean.parseBoolean(stringValue);
      else
         return defaultValue;
   }

   public static double loadDoubleProperty(String propertyKey, double defaultValue)
   {
      String stringValue = System.getProperty(propertyKey);

      if (stringValue != null)
      {
         try
         {
            return Double.parseDouble(stringValue);
         }
         catch (NumberFormatException e)
         {
            LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
         }
      }

      return defaultValue;
   }

   public static int loadIntegerProperty(String propertyKey, int defaultValue)
   {
      String stringValue = System.getProperty(propertyKey);

      if (stringValue != null)
      {
         try
         {
            return Integer.parseInt(stringValue);
         }
         catch (NumberFormatException e)
         {
            LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
         }
      }

      return defaultValue;
   }

   public static long loadLongProperty(String propertyKey, long defaultValue)
   {
      String stringValue = System.getProperty(propertyKey);

      if (stringValue != null)
      {
         try
         {
            return Long.parseLong(stringValue);
         }
         catch (NumberFormatException e)
         {
            LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
         }
      }

      return defaultValue;
   }

   public static String loadStringProperty(String propertyKey, String defaultValue)
   {
      String stringValue = System.getProperty(propertyKey);
      return stringValue != null ? stringValue : defaultValue;
   }

   public static <E extends Enum<E>> E loadEnumProperty(String propertyKey, Class<E> enumClass, E defaultValue)
   {
      String stringValue = System.getProperty(propertyKey);

      if (stringValue != null)
      {
         try
         {
            return parseEnumValue(enumClass, stringValue);
         }
         catch (IllegalArgumentException e)
         {
            LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
         }
      }

      return defaultValue;
   }

   /////////////////////////////////////////////////////////////////////////////////////////
   /////////////////// Loading Properties and/or Environment Variable //////////////////////
   /////////////////////////////////////////////////////////////////////////////////////////

   public static boolean loadBooleanPropertyOrEnvironment(String propertyKey, String environmentVariableName, boolean defaultValue)
   {
      String stringValue = null;

      if (propertyKey != null)
      {
         stringValue = System.getProperty(propertyKey);
         if (stringValue != null)
            return Boolean.parseBoolean(stringValue);
      }

      if (environmentVariableName != null)
      {
         stringValue = System.getenv(environmentVariableName);

         if (stringValue != null)
            return Boolean.parseBoolean(stringValue);
      }

      return defaultValue;
   }

   public static boolean loadBooleanPropertyOrEnvironment(String propertyKey, String environmentVariableName, boolean defaultValue, boolean definedNotSetValue)
   {
      String stringValue = null;

      if (propertyKey != null)
      {
         stringValue = System.getProperty(propertyKey);
         if (stringValue != null && stringValue.isEmpty())
            return definedNotSetValue;
         if (stringValue != null)
            return Boolean.parseBoolean(stringValue);
      }

      if (environmentVariableName != null)
      {
         stringValue = System.getenv(environmentVariableName);

         if (stringValue != null)
            return Boolean.parseBoolean(stringValue);
      }

      return defaultValue;
   }

   public static double loadDoublePropertyOrEnvironment(String propertyKey, String environmentVariableName, double defaultValue)
   {
      String stringValue = null;

      if (propertyKey != null)
      {
         stringValue = System.getProperty(propertyKey);
         if (stringValue != null)
         {
            try
            {
               return Double.parseDouble(stringValue);
            }
            catch (NumberFormatException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      if (environmentVariableName != null)
      {
         stringValue = System.getenv(environmentVariableName);

         if (stringValue != null)
         {
            try
            {
               return Double.parseDouble(stringValue);
            }
            catch (NumberFormatException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      return defaultValue;
   }

   public static int loadIntegerPropertyOrEnvironment(String propertyKey, String environmentVariableName, int defaultValue)
   {
      String stringValue = null;

      if (propertyKey != null)
      {
         stringValue = System.getProperty(propertyKey);
         if (stringValue != null)
         {
            try
            {
               return Integer.parseInt(stringValue);
            }
            catch (NumberFormatException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      if (environmentVariableName != null)
      {
         stringValue = System.getenv(environmentVariableName);

         if (stringValue != null)
         {
            try
            {
               return Integer.parseInt(stringValue);
            }
            catch (NumberFormatException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      return defaultValue;
   }

   public static long loadLongPropertyOrEnvironment(String propertyKey, String environmentVariableName, long defaultValue)
   {
      String stringValue = null;

      if (propertyKey != null)
      {
         stringValue = System.getProperty(propertyKey);
         if (stringValue != null)
         {
            try
            {
               return Long.parseLong(stringValue);
            }
            catch (NumberFormatException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      if (environmentVariableName != null)
      {
         stringValue = System.getenv(environmentVariableName);

         if (stringValue != null)
         {
            try
            {
               return Long.parseLong(stringValue);
            }
            catch (NumberFormatException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      return defaultValue;
   }

   public static String loadStringPropertyOrEnvironment(String propertyKey, String environmentVariableName, String defaultValue)
   {
      String stringValue = null;

      if (propertyKey != null)
      {
         stringValue = System.getProperty(propertyKey);
         if (stringValue != null)
            return stringValue;
      }

      if (environmentVariableName != null)
      {
         stringValue = System.getenv(environmentVariableName);

         if (stringValue != null)
            return stringValue;
      }

      return defaultValue;
   }

   public static <E extends Enum<E>> E loadEnumPropertyOrEnvironment(String propertyKey, String environmentVariableName, Class<E> enumClass, E defaultValue)
   {
      String stringValue = null;

      if (propertyKey != null)
      {
         stringValue = System.getProperty(propertyKey);
         if (stringValue != null)
         {
            try
            {
               return parseEnumValue(enumClass, stringValue);
            }
            catch (IllegalArgumentException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      if (environmentVariableName != null)
      {
         stringValue = System.getenv(environmentVariableName);

         if (stringValue != null)
         {
            try
            {
               return parseEnumValue(enumClass, stringValue);
            }
            catch (IllegalArgumentException e)
            {
               LogTools.error("Exception while loading property {}: {}. Using default value.", propertyKey, e.getMessage());
            }
         }
      }

      return defaultValue;
   }

   private static <E extends Enum<E>> E parseEnumValue(Class<E> enumClass, String stringValue)
   {
      try
      {
         return Enum.valueOf(enumClass, stringValue);
      }
      catch (IllegalArgumentException e)
      {
         for (E enumConstant : enumClass.getEnumConstants())
         {
            if (enumConstant.name().equalsIgnoreCase(stringValue))
               return enumConstant;
         }
         throw e;
      }
   }
}
