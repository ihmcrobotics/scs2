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

/**
 * Base class representing a template used to create 1+ yoGraphics. A yoGraphic is a 2D/3D graphic
 * object which properties can be backed by yoVariables allowing the graphics to move or change
 * during a simulation or session.
 * <p>
 * The {@code YoGraphicDefinition} is to be passed before initialization of a session (either before
 * starting a simulation or when creating a yoVariable server), such that the SCS GUI can use the
 * definitions and create the actual graphics.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public abstract class YoGraphicDefinition
{
   /**
    * Whether to print additional debug information when parsing {@code YoGraphicDefinition}s.
    * <p>
    * Can be set to {@code true} using the program argument {@code scs2.definition.debugParsing}.
    * </p>
    */
   private static final boolean DEBUG_PARSING;

   static
   {
      boolean debugParsingValue = false;
      String debugParsingProp = System.getProperty("scs2.definition.debugParsing");
      if (debugParsingProp != null)
         debugParsingValue = Boolean.parseBoolean(debugParsingProp);
      DEBUG_PARSING = debugParsingValue;
   }

   /** Human readable name for this yoGraphic, it will show up in the SCS GUI. */
   protected String name;
   /** Whether the yoGrpahic should be visible by default when created. */
   protected boolean visible = true;

   public YoGraphicDefinition()
   {
      registerStringField("name", this::getName, this::setName);
      registerBooleanField("visible", this::isVisible, this::setVisible);
   }

   /** Human readable name for this yoGraphic, it will show up in the SCS GUI. */
   @XmlAttribute
   public final void setName(String name)
   {
      this.name = name;
   }

   /** Whether the yoGrpahic should be visible by default when created. */
   @XmlAttribute
   public final void setVisible(boolean visible)
   {
      this.visible = visible;
   }

   /** Human readable name for this yoGraphic, it will show up in the SCS GUI. */
   public final String getName()
   {
      return name;
   }

   /** Whether the yoGrpahic should be visible by default when created. */
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
         if (!Objects.equals(name, other.name) || (visible != other.visible))
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

   /**
    * Provides a string representation of this definition while indenting it with as many "\t" as
    * desired.
    *
    * @param indent the number of tabulation to indent the resulting string.
    * @return the string.
    */
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

   /**
    * [For internal use] - Created an indented string representation of the given list where each of
    * its element is on a spearate line.
    *
    * @param indent          the number of tabulation to indent the resulting string.
    * @param useBrace        whether to use braces or brackets.
    * @param list            the list to create the string of.
    * @param elementToString the function used to get the string for each element.
    * @return the string.
    */
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

   /**
    * [For internal use] - Creates a simple string of this definition containing only the type and the
    * name.
    */
   protected final String toParsableString()
   {
      return getClass().getSimpleName() + "=" + name;
   }

   /**
    * [For internal use] - Parses a new empty yoGraphic definition from a string previously generated
    * using {@link #toParsableString()}.
    *
    * @param value the string representation of the definition to create.
    * @return the new definition which fields still need to be initialized.
    */
   @SuppressWarnings("unchecked")
   static YoGraphicDefinition parse(String value)
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

   /**
    * [For serializing/deserializing] - Creates for each {@code YoGraphicDefinition} in the trees
    * starting at the given {@code roots} a summary of its fields' name and {@code String} value and
    * returns the result as a list.
    * <p>
    * This exported summary list can be used to simplify serialization of {@code YoGraphicDefinition}
    * by using only lists and {@code String}s to represent all of the {@code YoGraphicDefinition}s in
    * the trees starting at {@code roots}.
    * </p>
    * <p>
    * The summary list can later be parsed back using {@link #parseTreeYoGraphicFieldsSummary(List)}
    * which is expected to return a deep copy of the original trees.
    * </p>
    *
    * @param roots a collection of the roots of trees to export the summary list of.
    * @return the summary list representing the {@code YoGraphicDefinition} trees. The first item in
    *         the returned list describes the first element of {@code roots}.
    */
   public static List<YoGraphicFieldsSummary> exportSubtreeYoGraphicFieldsSummaryList(Collection<YoGraphicGroupDefinition> roots)
   {
      List<YoGraphicFieldsSummary> output = new ArrayList<>();
      for (YoGraphicGroupDefinition root : roots)
      {
         exportSubtreeYoGraphicFieldsSummaryList(root, output);
      }
      return output;
   }

   /**
    * [For serializing/deserializing] - Creates for each {@code YoGraphicDefinition} in the tree
    * starting at the given {@code root} a summary of its fields' name and {@code String} value and
    * returns the result as a list.
    * <p>
    * This exported summary list can be used to simplify serialization of {@code YoGraphicDefinition}
    * by using only lists and {@code String}s to represent all of the {@code YoGraphicDefinition}s in
    * the tree starting at {@code root}.
    * </p>
    * <p>
    * The summary list can later be parsed back using {@link #parseTreeYoGraphicFieldsSummary(List)}
    * which is expected to return a deep copy of the original tree.
    * </p>
    *
    * @param root the root of the tree to export the summary list of.
    * @return the summary list representing the {@code YoGraphicDefinition} tree. The first item in the
    *         returned list describes the given {@code root}.
    */
   public static List<YoGraphicFieldsSummary> exportSubtreeYoGraphicFieldsSummaryList(YoGraphicGroupDefinition root)
   {
      return exportSubtreeYoGraphicFieldsSummaryList(root, null);
   }

   /**
    * [For internal use] - This method is used to enable iteration for exporting
    * {@code YoGraphicDefinition} summaries for multiple trees.
    *
    * @param root         the root of the tree to export the summary list of.
    * @param outputToPack the list to which the summaries are to be added. Can be {@code null}
    * @return {@code outputToPack} for convenience.
    */
   static List<YoGraphicFieldsSummary> exportSubtreeYoGraphicFieldsSummaryList(YoGraphicGroupDefinition root, List<YoGraphicFieldsSummary> outputToPack)
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

   /**
    * [For serializing/deserializing] - Parses the given summary list into trees of
    * {@code YoGraphicDefinition}s.
    *
    * @param treeYoGraphicFieldsSummaryList the summary list where each item represent one
    *                                       {@code YoGraphicDefinition}. The first element is expected
    *                                       to represent a {@code YoGraphicGroupDefinition}.
    * @return the root group for each tree parsed.
    */
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

   /**
    * [For internal use] - Parses the given summary list into a tree structure of
    * {@code YoGraphicDefinition}. The first element in the list is expected to describe a
    * {@code YoGraphicGroupDefinition}.
    *
    * @param start                          the group to start from that is also initialized with the
    *                                       first element form the summary list.
    * @param treeYoGraphicFieldsSummaryList the list of summaries describing every
    *                                       {@code YoGraphicDefinition} to be created and initialized.
    */
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
    * [For internal use] - Creates a map from field name to field value as {@code String} for every
    * field representing this {@code YoGraphicDefinition}.
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

   /**
    * [For internal use] - Parses the values of {@code this} from the given summary.
    *
    * @param fieldsSummary the summary containing the field values as {@code String} for {@code this}.
    */
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

   /**
    * [For internal use] - Convenience class for gathering the name, value as {@code String} generator,
    * and value parser for one field of one {@code YoGraphicDefinition}.
    *
    * @author Sylvain Bertrand
    */
   private static class FieldInfoConverter
   {
      private final String fieldName;
      private final Supplier<String> fieldValueSupplier;
      private final Consumer<String> fieldValueParser;

      /**
       * Creates a new converter for a field.
       *
       * @param fieldName          the name of the field of interest.
       * @param fieldValueSupplier the function used to generate the {@code String} value for the field.
       * @param fieldValueParser   the function used to parse a {@code String} into the actual field
       *                           value.
       */
      public FieldInfoConverter(String fieldName, Supplier<String> fieldValueSupplier, Consumer<String> fieldValueParser)
      {
         this.fieldName = fieldName;
         this.fieldValueSupplier = fieldValueSupplier;
         this.fieldValueParser = fieldValueParser;
      }
   }

   /**
    * Convenience class that pairs the name and {@code String} value for one field of a
    * {@code YoGraphicDefinition}.
    *
    * @author Sylvain Bertrand
    */
   public static class YoGraphicFieldInfo
   {
      private final String fieldName, fieldValue;

      public YoGraphicFieldInfo(String fieldName, String fieldValue)
      {
         this.fieldName = fieldName;
         this.fieldValue = fieldValue;
      }

      /**
       * The name of the field the value is for.
       *
       * @return the field name.
       */
      public String getFieldName()
      {
         return fieldName;
      }

      /**
       * The value as a {@code String} for the field.
       *
       * @return the field value as a {@code String}.
       */
      public String getFieldValue()
      {
         return fieldValue;
      }
   }

   /**
    * Convenience class for listing the name and value of all the fields of one
    * {@code YoGraphicDefinition}
    *
    * @author Sylvain Bertrand
    */
   public static class YoGraphicFieldsSummary extends ArrayList<YoGraphicFieldInfo>
   {
      private static final long serialVersionUID = -1654039568977911943L;

      public YoGraphicFieldsSummary()
      {
      }
   }

   /**
    * [For internal use] - Maps of all {@code this} fields together with converters to help with
    * generating {@code String} values for each field and parsing back the field value from a
    * {@code String} for each field.
    * <p>
    * This is used to enable summary export and parsing from summary which can be used to help with
    * serializing/deserializing {@code YoGraphicDefinition}s.
    * </p>
    */
   @XmlTransient
   private final Map<String, FieldInfoConverter> definitionFields = new LinkedHashMap<>();

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code List}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    * @param elementLabel     label used in the generated {@code String} value of the list.
    * @param elementToString  the {@code String} generator to use for each element.
    * @param elementParser    the element parser to use when parsing back the {@code String} value of
    *                         an element.
    */
   protected final <T> void registerListField(String fieldName,
                                              Supplier<List<T>> fieldValueGetter,
                                              Consumer<List<T>> fieldValueSetter,
                                              String elementLabel,
                                              Function<T, String> elementToString,
                                              Function<String, T> elementParser)
   {
      registerStringField(fieldName, () ->
      {
         List<T> value = fieldValueGetter.get();
         return value == null ? null : listToParsableString(value, elementLabel, elementToString);
      }, value -> fieldValueSetter.accept(parseList(value, elementLabel, elementParser)));
   }

   /**
    * [For internal use] - Generates a {@code String} representation of the given {@code list} that can
    * later be parsed back with {@link #parseList(String, String, Function)}.
    *
    * @param list            the list to generate the {@code String} of.
    * @param elementLabel    the label used in front of each element {@code String} representation,
    *                        e.g. "List(..., e11=elementAsString, ...)" where {@code elementLabel="e"}
    *                        in this example.
    * @param elementToString the {@code String} generator to use for each element.
    * @return a {@code String} representation of the {@code list} of the form:
    *         {@code "List(e0=element0AsString, e1=elementAsString,...)"}.
    */
   static <T> String listToParsableString(List<T> list, String elementLabel, Function<T, String> elementToString)
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

   /**
    * [For internal use] - Parses a list from the given {@code String} representation {@code value}. It
    * is assumed that {@code value} was generated using
    * {@link #listToParsableString(List, String, Function)}.
    *
    * @param value         the {@code String} representation of the list to be parsed.
    * @param elementLabel  the label used in front of each element {@code String} representation, e.g.
    *                      "List(..., e11=elementAsString, ...)" where {@code elementLabel="e"} in this
    *                      example.
    * @param elementParser the element parser to use when parsing back the {@code String} value of an
    *                      element.
    * @return the parsed list.
    */
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

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code YoListDefinition}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerYoListField(String fieldName, Supplier<YoListDefinition> fieldValueGetter, Consumer<YoListDefinition> fieldValueSetter)
   {
      registerStringField(fieldName, () -> Objects.toString(fieldValueGetter.get(), null), value -> fieldValueSetter.accept(YoListDefinition.parse(value)));
   }

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code YoTuple2DDefinition}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerTuple2DField(String fieldName, Supplier<YoTuple2DDefinition> fieldValueGetter, Consumer<YoTuple2DDefinition> fieldValueSetter)
   {
      registerStringField(fieldName, () -> Objects.toString(fieldValueGetter.get(), null), value -> fieldValueSetter.accept(YoTuple2DDefinition.parse(value)));
   }

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code YoTuple3DDefinition}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerTuple3DField(String fieldName, Supplier<YoTuple3DDefinition> fieldValueGetter, Consumer<YoTuple3DDefinition> fieldValueSetter)
   {
      registerStringField(fieldName, () -> Objects.toString(fieldValueGetter.get(), null), value -> fieldValueSetter.accept(YoTuple3DDefinition.parse(value)));
   }

   /**
    * [For internal use] - Registers a field for {@code this} of the type
    * {@code YoOrientation3DDefinition}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerOrientation3DField(String fieldName,
                                                   Supplier<YoOrientation3DDefinition> fieldValueGetter,
                                                   Consumer<YoOrientation3DDefinition> fieldValueSetter)
   {
      registerStringField(fieldName,
                          () -> Objects.toString(fieldValueGetter.get(), null),
                          value -> fieldValueSetter.accept(YoOrientation3DDefinition.parse(value)));
   }

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code PaintDefinition}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerPaintField(String fieldName, Supplier<PaintDefinition> fieldValueGetter, Consumer<PaintDefinition> fieldValueSetter)
   {
      registerStringField(fieldName, () -> Objects.toString(fieldValueGetter.get(), null), value -> fieldValueSetter.accept(PaintDefinition.parse(value)));
   }

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code double}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerDoubleField(String fieldName, DoubleSupplier fieldValueGetter, DoubleConsumer fieldValueSetter)
   {
      registerStringField(fieldName, () -> Double.toString(fieldValueGetter.getAsDouble()), string -> fieldValueSetter.accept(Double.parseDouble(string)));
   }

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code boolean}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerBooleanField(String fieldName, BooleanSupplier fieldValueGetter, Consumer<Boolean> fieldValueSetter)
   {
      registerStringField(fieldName, () -> Boolean.toString(fieldValueGetter.getAsBoolean()), string -> fieldValueSetter.accept(Boolean.valueOf(string)));
   }

   /**
    * [For internal use] - Registers a field for {@code this} of the type {@code boolean}.
    *
    * @param fieldName        the name of the field.
    * @param fieldValueGetter the getter associated to that field.
    * @param fieldValueSetter the setter associated to that field.
    */
   protected final void registerStringField(String fieldName, Supplier<String> fieldValueGetter, Consumer<String> fieldValueSetter)
   {
      definitionFields.put(fieldName, new FieldInfoConverter(fieldName, fieldValueGetter, fieldValueSetter));
   }
}
