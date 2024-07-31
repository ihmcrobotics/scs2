package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoGraphic;

import javafx.beans.InvalidationListener;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.ReadOnlyObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableBooleanValue;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import org.reflections.Reflections;
import org.reflections.scanners.SubTypesScanner;
import org.reflections.util.ClasspathHelper;
import org.reflections.util.ConfigurationBuilder;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.ReferenceFrameWrapper;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.CompositePropertyTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple2DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.Tuple3DProperty;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX2D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFX3D;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicFXItem;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGraphicTools;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoGroupFX;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.BiPredicate;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class YoGraphicFXControllerTools
{
   public static List<Class<? extends YoGraphicFXItem>> yoGraphicFXTypes;
   public static List<Class<? extends YoGraphicFX2D>> yoGraphicFX2DTypes;
   public static List<Class<? extends YoGraphicFX3D>> yoGraphicFX3DTypes;

   static
   {
      Thread loader = new Thread(() ->
                                 {
                                    Reflections reflections = new Reflections(new ConfigurationBuilder().setUrls(ClasspathHelper.forPackage(YoGraphicFXItem.class.getPackage()
                                                                                                                                                                 .getName()))
                                                                                                        .setScanners(new SubTypesScanner()));
                                    Set<Class<? extends YoGraphicFXItem>> yoGraphicFXSubTypes = reflections.getSubTypesOf(YoGraphicFXItem.class);
                                    yoGraphicFXTypes = yoGraphicFXSubTypes.stream()
                                                                          .filter(type -> !Modifier.isAbstract(type.getModifiers()) && !type.isInterface())
                                                                          .sorted((c1, c2) -> c1.getSimpleName().compareTo(c2.getSimpleName()))
                                                                          .collect(Collectors.toList());
                                    Set<Class<? extends YoGraphicFX2D>> yoGraphicFX2DSubTypes = reflections.getSubTypesOf(YoGraphicFX2D.class);
                                    yoGraphicFX2DTypes = yoGraphicFX2DSubTypes.stream()
                                                                              .filter(type -> !Modifier.isAbstract(type.getModifiers()) && !type.isInterface())
                                                                              .sorted((c1, c2) -> c1.getSimpleName().compareTo(c2.getSimpleName()))
                                                                              .collect(Collectors.toList());
                                    Set<Class<? extends YoGraphicFX3D>> yoGraphicFX3DSubTypes = reflections.getSubTypesOf(YoGraphicFX3D.class);
                                    yoGraphicFX3DTypes = yoGraphicFX3DSubTypes.stream()
                                                                              .filter(type -> !Modifier.isAbstract(type.getModifiers()) && !type.isInterface())
                                                                              .sorted((c1, c2) -> c1.getSimpleName().compareTo(c2.getSimpleName()))
                                                                              .collect(Collectors.toList());
                                 }, "YoGraphicFX Loader");
      loader.setPriority(Thread.MIN_PRIORITY);
      loader.setDaemon(true);
      loader.start();
   }

   public static void loadResources()
   {
      // Only need to load this class to get the resources loaded.
   }

   public static YoGraphicFXItem duplicateYoGraphicFXItemAndRegister(YoGraphicFXItem itemToDuplicate)
   {
      String cloneName = createAvailableYoGraphicFXItemName(itemToDuplicate.getRootGroup(),
                                                            itemToDuplicate.getNamespace(),
                                                            itemToDuplicate.getName(),
                                                            itemToDuplicate.getClass());
      YoGraphicFXItem clone = itemToDuplicate.clone();
      clone.setName(cloneName);
      itemToDuplicate.getParentGroup().addYoGraphicFXItem(clone);
      return clone;
   }

   public static YoGraphicFXItem createYoGraphicFXItemAndRegister(ReferenceFrameWrapper worldFrame,
                                                                  YoGroupFX parentGroup,
                                                                  String itemName,
                                                                  Class<? extends YoGraphicFXItem> itemTypeToInstantiate)
   {
      if (itemTypeToInstantiate == YoGroupFX.class)
      {
         YoGroupFX item = new YoGroupFX(itemName);
         boolean success = parentGroup.addChild(item);
         return success ? item : null;
      }
      else if (YoGraphicFX.class.isAssignableFrom(itemTypeToInstantiate))
      {
         @SuppressWarnings("unchecked") YoGraphicFX item = newInstance(worldFrame, (Class<? extends YoGraphicFX>) itemTypeToInstantiate);

         item.setName(itemName);
         boolean success = parentGroup.addYoGraphicFXItem(item);
         return success ? item : null;
      }
      else
      {
         throw new RuntimeException("Unexpected item type: " + itemTypeToInstantiate.getSimpleName());
      }
   }

   private static YoGraphicFX newInstance(ReferenceFrameWrapper worldFrame, Class<? extends YoGraphicFX> itemTypeToInstantiate)
   {
      try
      {
         Constructor<? extends YoGraphicFX> constructor = itemTypeToInstantiate.getConstructor(ReferenceFrameWrapper.class);
         return constructor.newInstance(worldFrame);
      }
      catch (NoSuchMethodException | SecurityException | InstantiationException | IllegalAccessException | IllegalArgumentException |
             InvocationTargetException e)
      {
         throw new RuntimeException("Something went wrong when instantiating a YoGraphicFX attempting to invoke its constructor with ReferenceFrame argument: ",
                                    e);
      }
   }

   public static <G extends YoGraphicFXItem> String createAvailableYoGraphicFXItemName(YoGroupFX root, String namespace, String initialName, Class<G> itemType)
   {
      if (YoGraphicFX2D.class.isAssignableFrom(itemType))
         return createAvailableYoGraphicFX2DName(root, namespace, initialName);
      else if (YoGraphicFX3D.class.isAssignableFrom(itemType))
         return createAvailableYoGraphicFX3DName(root, namespace, initialName);
      else if (YoGroupFX.class.isAssignableFrom(itemType))
         return createAvailableYoGraphicFXGroupName(root, namespace, initialName);
      else
         throw new RuntimeException("Unexpected item type: " + itemType.getSimpleName());
   }

   public static String createAvailableYoGraphicFX2DName(YoGroupFX root, String namespace, String initialName)
   {
      return createAvailableYoGraphicFXItemName(root, namespace, initialName, YoGroupFX::containsYoGraphicFX2D);
   }

   public static String createAvailableYoGraphicFX3DName(YoGroupFX root, String namespace, String initialName)
   {
      return createAvailableYoGraphicFXItemName(root, namespace, initialName, YoGroupFX::containsYoGraphicFX3D);
   }

   public static String createAvailableYoGraphicFXGroupName(YoGroupFX root, String namespace, String initialName)
   {
      return createAvailableYoGraphicFXItemName(root, namespace, initialName, YoGroupFX::containsChild);
   }

   private static String createAvailableYoGraphicFXItemName(YoGroupFX root,
                                                            String namespace,
                                                            String initialName,
                                                            BiPredicate<YoGroupFX, String> doesNameExistFunction)
   {
      YoGroupFX group = YoGraphicTools.findYoGraphicFXGroup(root, namespace);

      if (group == null)
         return initialName;

      String guessName = initialName;

      int startIndex = 0;

      String[] numbers = initialName.split("[^0-9]+");

      if (numbers.length > 0)
      {
         String lastNumber = numbers[numbers.length - 1];

         if (initialName.endsWith(lastNumber))
         {
            startIndex = Integer.parseInt(lastNumber);
            initialName = initialName.substring(0, initialName.length() - lastNumber.length());
         }
      }

      for (int i = startIndex; i < Integer.MAX_VALUE; i++)
      {
         if (!doesNameExistFunction.test(group, guessName))
            return guessName;

         guessName = initialName + Integer.toString(i + 1);
      }

      return null;
   }

   public static <G extends YoGraphicFXItem> String createAvailableYoGraphicFXItemName(YoGroupFX parentGroup, String initialName, Class<G> itemType)
   {
      if (YoGraphicFX2D.class.isAssignableFrom(itemType))
         return createAvailableYoGraphicFX2DName(parentGroup, initialName);
      else if (YoGraphicFX3D.class.isAssignableFrom(itemType))
         return createAvailableYoGraphicFX3DName(parentGroup, initialName);
      else if (YoGroupFX.class.isAssignableFrom(itemType))
         return createAvailableYoGraphicFXGroupName(parentGroup, initialName);
      else
         throw new RuntimeException("Unexpected item type: " + itemType.getSimpleName());
   }

   public static String createAvailableYoGraphicFX2DName(YoGroupFX parentGroup)
   {
      return createAvailableYoGraphicFX2DName(parentGroup, YoGraphicFX2D.class.getSimpleName());
   }

   public static String createAvailableYoGraphicFX2DName(YoGroupFX parentGroup, String initialName)
   {
      return createAvailableYoGraphicFXItemName(parentGroup, initialName, YoGroupFX::containsYoGraphicFX2D);
   }

   public static String createAvailableYoGraphicFX3DName(YoGroupFX parentGroup)
   {
      return createAvailableYoGraphicFX3DName(parentGroup, YoGraphicFX3D.class.getSimpleName());
   }

   public static String createAvailableYoGraphicFX3DName(YoGroupFX parentGroup, String initialName)
   {
      return createAvailableYoGraphicFXItemName(parentGroup, initialName, YoGroupFX::containsYoGraphicFX3D);
   }

   public static String createAvailableYoGraphicFXGroupName(YoGroupFX parentGroup)
   {
      return createAvailableYoGraphicFXGroupName(parentGroup, YoGroupFX.class.getSimpleName());
   }

   public static String createAvailableYoGraphicFXGroupName(YoGroupFX parentGroup, String initialName)
   {
      return createAvailableYoGraphicFXItemName(parentGroup, initialName, YoGroupFX::containsChild);
   }

   private static String createAvailableYoGraphicFXItemName(YoGroupFX parentGroup, String initialName, BiPredicate<YoGroupFX, String> doesNameExistFunction)
   {
      if (parentGroup == null)
         return initialName;

      String guessName = initialName;

      int startIndex = 0;

      String[] numbers = initialName.split("[^0-9]+");

      if (numbers.length > 0)
      {
         String lastNumber = numbers[numbers.length - 1];

         if (initialName.endsWith(lastNumber))
         {
            startIndex = Integer.parseInt(lastNumber);
            initialName = initialName.substring(0, initialName.length() - lastNumber.length());
         }
      }

      for (int i = startIndex; i < Integer.MAX_VALUE; i++)
      {
         if (!doesNameExistFunction.test(parentGroup, guessName))
            return guessName;

         guessName = initialName + Integer.toString(i + 1);
      }

      return null;
   }

   public static ChangeListener<String> numericalValidityBinding(StringProperty stringProperty, BooleanProperty validityProperty)
   {
      ChangeListener<String> listener = (observable, oldValue, newValue) -> validityProperty.set(CompositePropertyTools.isParsableAsDouble(newValue));
      stringProperty.addListener(listener);
      return listener;
   }

   public static InvalidationListener fullnameValidityBinding(StringProperty nameProperty,
                                                              StringProperty namespaceProperty,
                                                              BooleanProperty validityProperty,
                                                              YoGroupFX rootGroup,
                                                              YoGraphicFXItem yoGraphicFXItem)
   {
      Supplier<YoGraphicFXItem> searchBasedYoGraphicSupplier = () -> YoGraphicTools.findYoGraphicFXItem(rootGroup,
                                                                                                        namespaceProperty.get(),
                                                                                                        nameProperty.get(),
                                                                                                        yoGraphicFXItem.getClass());
      return fullnameValidityBinding(nameProperty, namespaceProperty, validityProperty, yoGraphicFXItem, searchBasedYoGraphicSupplier);
   }

   private static <T> InvalidationListener fullnameValidityBinding(StringProperty nameProperty,
                                                                   StringProperty namespaceProperty,
                                                                   BooleanProperty validityProperty,
                                                                   T yoGraphicFX,
                                                                   Supplier<T> searchBasedYoGraphicSupplier)
   {
      InvalidationListener listener = observable ->
      {
         if (nameProperty.get() == null || nameProperty.get().isEmpty())
         {
            validityProperty.set(false);
         }
         else
         {
            T searchResult = searchBasedYoGraphicSupplier.get();
            validityProperty.set(searchResult == null || searchResult == yoGraphicFX);
         }
      };
      listener.invalidated(null);
      nameProperty.addListener(listener);
      namespaceProperty.addListener(listener);
      return listener;
   }

   public static ChangeListener<Boolean> bindValidityImageView(ObservableBooleanValue observableBoolean, ImageView imageView)
   {
      return bindBooleanToImageView(observableBoolean, imageView, SessionVisualizerIOTools.VALID_ICON_IMAGE, SessionVisualizerIOTools.INVALID_ICON_IMAGE);
   }

   public static ChangeListener<Boolean> bindBooleanToImageView(ObservableBooleanValue observableBoolean,
                                                                ImageView imageView,
                                                                Image imageWhenTrue,
                                                                Image imageWhenFalse)
   {
      updateImageView(observableBoolean.get(), imageView, imageWhenTrue, imageWhenFalse);

      ChangeListener<Boolean> listener = (observable, oldValue, newValue) ->
      {
         if (newValue.equals(oldValue))
            return;
         else
            updateImageView(newValue.booleanValue(), imageView, imageWhenTrue, imageWhenFalse);
      };
      observableBoolean.addListener(listener);
      return listener;
   }

   public static void updateImageView(boolean value, ImageView imageView, Image imageWhenTrue, Image imageWhenFalse)
   {
      imageView.setImage(value ? imageWhenTrue : imageWhenFalse);
   }

   public static String replaceAndMatchCase(String original, String search, String replacement)
   {
      int indexOf = original.toLowerCase().indexOf(search.toLowerCase());
      if (indexOf == -1)
         return original;

      if (Character.isAlphabetic(original.charAt(indexOf)))
      {
         if (Character.isUpperCase(original.charAt(indexOf)))
            replacement = Character.toUpperCase(replacement.charAt(0)) + replacement.substring(1);
         else
            replacement = Character.toLowerCase(replacement.charAt(0)) + replacement.substring(1);
      }

      return original.replaceAll("(?i)" + search, replacement);
   }

   public static ReadOnlyObjectProperty<List<DoubleProperty>> toSingletonDoubleSupplierListProperty(ReadOnlyObjectProperty<List<DoubleProperty[]>> inputProperty)
   {
      ObjectProperty<List<DoubleProperty>> output = new SimpleObjectProperty<>(null, "singletonSupplierList", null);
      inputProperty.addListener((o, oldValue, newValue) -> output.set(newValue.stream().map(array -> array[0]).collect(Collectors.toList())));
      return output;
   }

   public static ReadOnlyObjectProperty<List<Tuple2DProperty>> toTuple2DDoubleSupplierListProperty(ReadOnlyObjectProperty<List<DoubleProperty[]>> inputProperty)
   {
      ObjectProperty<List<Tuple2DProperty>> output = new SimpleObjectProperty<>(null, "tuple2DSupplierList", null);
      inputProperty.addListener((o, oldValue, newValue) -> output.set(newValue.stream()
                                                                              .map(array -> new Tuple2DProperty(null, array))
                                                                              .collect(Collectors.toList())));
      return output;
   }

   public static ReadOnlyObjectProperty<List<Tuple3DProperty>> toTuple3DDoubleSupplierListProperty(ReadOnlyObjectProperty<List<DoubleProperty[]>> inputProperty)
   {
      ObjectProperty<List<Tuple3DProperty>> output = new SimpleObjectProperty<>(null, "tuple3DSupplierList", null);
      inputProperty.addListener((o, oldValue, newValue) -> output.set(newValue.stream()
                                                                              .map(array -> new Tuple3DProperty(null, array))
                                                                              .collect(Collectors.toList())));
      return output;
   }

   public static List<String[]> convertSingletonsToArrays(List<String> definitions)
   {
      if (definitions == null)
         return Collections.emptyList();
      else
         return definitions.stream().map(string -> new String[] {string}).collect(Collectors.toList());
   }
}
