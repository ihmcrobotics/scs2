package us.ihmc.scs2.definition.yoGraphic;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlTransient;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.visual.PaintDefinition;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;

public abstract class YoGraphicDefinition
{
   private static final boolean DEBUG_PARSING;

   static
   {
      boolean debugParsingValue = false;
      String debugParsingProp = System.getProperty("scs2.definition.debugParsing");
      if (debugParsingProp != null)
         debugParsingValue = Boolean.parseBoolean(debugParsingProp);
      DEBUG_PARSING = debugParsingValue;
   }

   protected String name;
   protected boolean visible = true;

   public YoGraphicDefinition()
   {
      registerField("name", this::getName, this::setName);
      registerField("visible", this::isVisible, this::setVisible);
   }

   @XmlAttribute
   public final void setName(String name)
   {
      this.name = name;
   }

   @XmlAttribute
   public final void setVisible(boolean visible)
   {
      this.visible = visible;
   }

   public final String getName()
   {
      return name;
   }

   public final boolean isVisible()
   {
      return visible;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoGraphicDefinition other)
      {
         if (!Objects.equals(name, other.name))
            return false;
         if (visible != other.visible)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public final String toString()
   {
      return toString(0);
   }

   public String toString(int indent)
   {
      String out = getClass().getSimpleName() + "[";
      boolean first = true;
      for (FieldInfoConverter fieldInfo : definitionFields.values())
      {
         if (!first)
            out += ", ";
         out += fieldInfo.fieldName + "=" + fieldInfo.fieldValueSupplier.get();
         first = false;
      }
      out += "]";
      return out;
   }

   static <T> String indentedListString(int indent, boolean useBrace, List<T> list, Function<T, String> elementToString)
   {
      if (list == null)
         return "null";
      if (list.isEmpty())
         return useBrace ? "{}" : "[]";

      String openingCharacter = useBrace ? "{" : "[";
      Object closingCharacter = useBrace ? "}" : "]";

      String prefix = openingCharacter + "\n" + "\t".repeat(indent + 1);
      String suffix = "\n" + "\t".repeat(indent) + closingCharacter;
      String separator = "\n" + "\t".repeat(indent + 1);
      return EuclidCoreIOTools.getCollectionString(prefix, suffix, separator, list, elementToString);
   }

   public final String toParsableString()
   {
      return getClass().getSimpleName() + "=" + name;
   }

   @SuppressWarnings("unchecked")
   public static YoGraphicDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith("YoGraphic"))
      {
         int equalsIndex = value.indexOf("=");
         String className = value.substring(0, equalsIndex);
         String name = value.substring(equalsIndex + 1);
         Class<? extends YoGraphicDefinition> definitionClass;
         try
         {
            String fullClassName = "%s.%s".formatted(YoGraphicDefinition.class.getPackageName(), className);
            definitionClass = (Class<? extends YoGraphicDefinition>) Class.forName(fullClassName);
            YoGraphicDefinition definition = definitionClass.getDeclaredConstructor().newInstance();
            definition.setName(name);
            return definition;
         }
         catch (Exception e)
         {
            throw new IllegalArgumentException("Unexpected yoGraphic definition format: " + value, e);
         }
      }
      else
      {
         throw new IllegalArgumentException("Unexpected yoGraphic definition format: " + value);
      }
   }

   public static List<YoGraphicFieldsSummary> exportSubtreeYoGraphicFieldsSummaryList(Collection<YoGraphicGroupDefinition> roots)
   {
      List<YoGraphicFieldsSummary> output = new ArrayList<>();
      for (YoGraphicGroupDefinition root : roots)
      {
         exportSubtreeYoGraphicFieldsSummaryList(root, output);
      }
      return output;
   }

   public static List<YoGraphicFieldsSummary> exportSubtreeYoGraphicFieldsSummaryList(YoGraphicGroupDefinition root)
   {
      return exportSubtreeYoGraphicFieldsSummaryList(root, null);
   }

   public static List<YoGraphicFieldsSummary> exportSubtreeYoGraphicFieldsSummaryList(YoGraphicGroupDefinition root, List<YoGraphicFieldsSummary> outputToPack)
   {
      if (root == null)
         return null;

      root.unwrapLists();

      if (outputToPack == null)
         outputToPack = new ArrayList<>();
      outputToPack.add(root.exportYoGraphicFieldsSummary());

      List<YoGraphicDefinition> children = root.getChildren();

      if (children != null)
      {
         for (int i = 0; i < children.size(); i++)
         {
            YoGraphicDefinition child = children.get(i);
            if (child instanceof YoGraphicGroupDefinition subGroup)
            {
               outputToPack.addAll(exportSubtreeYoGraphicFieldsSummaryList(subGroup));
            }
            else if (child instanceof YoGraphicListDefinition list)
            {
               if (list.getYoGraphics() != null)
               {
                  for (int j = 0; j < list.getYoGraphics().size(); j++)
                  {
                     outputToPack.add(list.getYoGraphics().get(j).exportYoGraphicFieldsSummary());
                  }
               }
            }
            else
            {
               outputToPack.add(child.exportYoGraphicFieldsSummary());
            }
         }
      }
      return outputToPack;
   }

   public static List<YoGraphicGroupDefinition> parseTreeYoGraphicFieldsSummary(List<YoGraphicFieldsSummary> treeYoGraphicFieldsSummaryList)
   {
      if (treeYoGraphicFieldsSummaryList == null)
         return null;

      treeYoGraphicFieldsSummaryList = new LinkedList<>(treeYoGraphicFieldsSummaryList);
      List<YoGraphicGroupDefinition> parsed = new ArrayList<>();
      while (!treeYoGraphicFieldsSummaryList.isEmpty())
      {
         YoGraphicGroupDefinition rootGroup = new YoGraphicGroupDefinition();
         parseTreeFieldValueInfoRecursive(rootGroup, treeYoGraphicFieldsSummaryList);
         parsed.add(rootGroup);
      }
      return parsed;
   }

   private static void parseTreeFieldValueInfoRecursive(YoGraphicGroupDefinition start, List<YoGraphicFieldsSummary> treeYoGraphicFieldsSummaryList)
   {
      start.parseYoGraphicFieldsInfo(treeYoGraphicFieldsSummaryList.remove(0));

      List<YoGraphicDefinition> children = start.getChildren();

      if (children != null)
      {
         for (int i = 0; i < children.size(); i++)
         {
            YoGraphicDefinition child = children.get(i);

            if (child instanceof YoGraphicGroupDefinition subGroup)
               parseTreeFieldValueInfoRecursive(subGroup, treeYoGraphicFieldsSummaryList);
            else
               child.parseYoGraphicFieldsInfo(treeYoGraphicFieldsSummaryList.remove(0));
         }
      }
   }

   /**
    * Creates a map from field name to field value as {@code String} for every field representing this
    * {@code YoGraphicDefinition}.
    * 
    * @return the map of field names to respective values.
    */
   YoGraphicFieldsSummary exportYoGraphicFieldsSummary()
   {
      YoGraphicFieldsSummary out = new YoGraphicFieldsSummary();

      for (FieldInfoConverter definitionField : definitionFields.values())
      {
         String value = definitionField.fieldValueSupplier.get();
         if (value != null)
            out.add(new YoGraphicFieldInfo(definitionField.fieldName, value));
      }
      return out;
   }

   void parseYoGraphicFieldsInfo(YoGraphicFieldsSummary fieldsSummary)
   {
      for (YoGraphicFieldInfo fieldNameStringValueEntry : fieldsSummary)
      {
         FieldInfoConverter field = definitionFields.get(fieldNameStringValueEntry.getFieldName());
         if (field == null)
         {
            if (DEBUG_PARSING)
               LogTools.error("Could not find field: {} for type: {}", fieldNameStringValueEntry.getFieldValue(), getClass().getSimpleName());
            continue;
         }

         try
         {
            field.fieldValueParser.accept(fieldNameStringValueEntry.getFieldValue());
         }
         catch (Exception e)
         {
            throw new RuntimeException("Error for definition: %s, field: %s, value: %s".formatted(getClass().getSimpleName(),
                                                                                                  field.fieldName,
                                                                                                  fieldNameStringValueEntry.getFieldValue()),
                                       e);
         }
      }
   }

   private static class FieldInfoConverter
   {
      private final String fieldName;
      private final Supplier<String> fieldValueSupplier;
      private final Consumer<String> fieldValueParser;

      public FieldInfoConverter(String fieldName, Supplier<String> fieldValueSupplier, Consumer<String> fieldValueParser)
      {
         this.fieldName = fieldName;
         this.fieldValueSupplier = fieldValueSupplier;
         this.fieldValueParser = fieldValueParser;
      }
   }

   public static class YoGraphicFieldInfo
   {
      private final String fieldName, fieldValue;

      public YoGraphicFieldInfo(String fieldName, String fieldValue)
      {
         this.fieldName = fieldName;
         this.fieldValue = fieldValue;
      }

      public String getFieldName()
      {
         return fieldName;
      }

      public String getFieldValue()
      {
         return fieldValue;
      }
   }

   public static class YoGraphicFieldsSummary extends ArrayList<YoGraphicFieldInfo>
   {
      private static final long serialVersionUID = -1654039568977911943L;

      public YoGraphicFieldsSummary()
      {
      }
   }

   @XmlTransient
   private final Map<String, FieldInfoConverter> definitionFields = new LinkedHashMap<>();

   protected final <T> void registerListField(String fieldName,
                                              Supplier<List<T>> fieldListValueGetter,
                                              Consumer<List<T>> fieldListValueSetter,
                                              String elementLabel,
                                              Function<T, String> elementToString,
                                              Function<String, T> elementParser)
   {
      registerField(fieldName, () ->
      {
         List<T> value = fieldListValueGetter.get();
         return value == null ? null : listToString(value, elementLabel, elementToString);
      }, value -> fieldListValueSetter.accept(parseList(value, elementLabel, elementParser)));
   }

   static <T> String listToString(List<T> list, String elementLabel, Function<T, String> elementToString)
   {
      if (list == null)
         return null;

      StringBuilder sb = new StringBuilder("List(");
      for (int i = 0; i < list.size(); i++)
      {
         if (i > 0)
            sb.append(", ");
         sb.append(elementLabel).append(i).append("=").append(elementToString.apply(list.get(i)));
      }
      sb.append(")");
      return sb.toString();
   }

   static <T> List<T> parseList(String value, String elementLabel, Function<String, T> elementParser)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith("List"))
      {
         String elementsSustring = value.substring(5, value.length() - 1).trim();
         ArrayList<T> list = new ArrayList<>();
         if (elementsSustring.isEmpty())
            return list;

         elementsSustring = elementsSustring.substring(elementLabel.length() + 2).trim();

         int nextElementIndex = 1;
         while (true)
         {
            String nextElementLabel = ", %s%d=".formatted(elementLabel, nextElementIndex);
            int indexOfLabel = elementsSustring.indexOf(nextElementLabel);

            String element;
            if (indexOfLabel != -1)
            {
               element = elementsSustring.substring(0, indexOfLabel);
               list.add(elementParser.apply(element));
               elementsSustring = elementsSustring.substring(indexOfLabel + nextElementLabel.length());
               nextElementIndex++;
            }
            else
            {
               list.add(elementParser.apply(elementsSustring));
               break;
            }
         }
         return list;
      }
      else
      {
         throw new IllegalArgumentException("Unknown list format: " + value);
      }
   }

   protected final void registerYoListField(String fieldName, Supplier<YoListDefinition> fieldListValueGetter, Consumer<YoListDefinition> fieldListValueSetter)
   {
      registerField(fieldName, () -> Objects.toString(fieldListValueGetter.get(), null), value -> fieldListValueSetter.accept(YoListDefinition.parse(value)));
   }

   protected final void registerTuple2DField(String fieldName,
                                             Supplier<YoTuple2DDefinition> fieldTuple2DValueGetter,
                                             Consumer<YoTuple2DDefinition> fieldTuple2DValueSetter)
   {
      registerField(fieldName,
                    () -> Objects.toString(fieldTuple2DValueGetter.get(), null),
                    value -> fieldTuple2DValueSetter.accept(YoTuple2DDefinition.parse(value)));
   }

   protected final void registerTuple3DField(String fieldName,
                                             Supplier<YoTuple3DDefinition> fieldTuple3DValueGetter,
                                             Consumer<YoTuple3DDefinition> fieldTuple3DValueSetter)
   {
      registerField(fieldName,
                    () -> Objects.toString(fieldTuple3DValueGetter.get(), null),
                    value -> fieldTuple3DValueSetter.accept(YoTuple3DDefinition.parse(value)));
   }

   protected final void registerOrientation3DField(String fieldName,
                                                   Supplier<YoOrientation3DDefinition> fieldOrientation3DValueGetter,
                                                   Consumer<YoOrientation3DDefinition> fieldOrientation3DValueSetter)
   {
      registerField(fieldName,
                    () -> Objects.toString(fieldOrientation3DValueGetter.get(), null),
                    value -> fieldOrientation3DValueSetter.accept(YoOrientation3DDefinition.parse(value)));
   }

   protected final void registerPaintField(String fieldName, Supplier<PaintDefinition> fieldPaintValueGetter, Consumer<PaintDefinition> fieldPaintValueSetter)
   {
      registerField(fieldName, () -> Objects.toString(fieldPaintValueGetter.get(), null), value -> fieldPaintValueSetter.accept(PaintDefinition.parse(value)));
   }

   protected final void registerField(String fieldName, DoubleSupplier fieldDoubleValueGetter, DoubleConsumer fieldDoubleValueSetter)
   {
      registerField(fieldName,
                    () -> Double.toString(fieldDoubleValueGetter.getAsDouble()),
                    string -> fieldDoubleValueSetter.accept(Double.parseDouble(string)));
   }

   protected final void registerField(String fieldName, BooleanSupplier fieldBooleanValueGetter, Consumer<Boolean> fieldBooleanValueSetter)
   {
      registerField(fieldName,
                    () -> Boolean.toString(fieldBooleanValueGetter.getAsBoolean()),
                    string -> fieldBooleanValueSetter.accept(Boolean.valueOf(string)));
   }

   protected final void registerField(String fieldName, Supplier<String> fieldStringValueSupplier, Consumer<String> fieldStringValueParser)
   {
      definitionFields.put(fieldName, new FieldInfoConverter(fieldName, fieldStringValueSupplier, fieldStringValueParser));
   }
}
