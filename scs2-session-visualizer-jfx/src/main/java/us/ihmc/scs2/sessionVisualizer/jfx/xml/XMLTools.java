package us.ihmc.scs2.sessionVisualizer.jfx.xml;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;

import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.configuration.SCSGuiConfigurationDefinition;
import us.ihmc.scs2.definition.yoComposite.YoCompositePatternListDefinition;
import us.ihmc.scs2.definition.yoEntry.YoEntryConfigurationDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.scs2.definition.yoSlider.YoSliderboardListDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositePattern;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;

public class XMLTools
{
   public static SCSGuiConfigurationDefinition loadSCSGuiConfigurationDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(SCSGuiConfigurationDefinition.class);
         Unmarshaller unmarshaller = context.createUnmarshaller();
         return (SCSGuiConfigurationDefinition) unmarshaller.unmarshal(inputStream);
      }
      finally
      {
         inputStream.close();
      }
   }

   public static List<YoCompositePattern> loadYoCompositePatterns(InputStream inputStream) throws JAXBException, IOException
   {
      return YoCompositeTools.toYoCompositePatterns(loadYoCompositePatternListDefinition(inputStream));
   }

   public static YoGraphicListDefinition loadYoGraphicListDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      return DefinitionIOTools.loadYoGraphicListDefinition(inputStream);
   }

   public static YoCompositePatternListDefinition loadYoCompositePatternListDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(YoCompositePatternListDefinition.class);
         Unmarshaller unmarshaller = context.createUnmarshaller();
         return (YoCompositePatternListDefinition) unmarshaller.unmarshal(inputStream);
      }
      finally
      {
         inputStream.close();
      }
   }

   public static YoEntryConfigurationDefinition loadYoEntryConfigurationDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(YoEntryConfigurationDefinition.class);
         Unmarshaller unmarshaller = context.createUnmarshaller();
         return (YoEntryConfigurationDefinition) unmarshaller.unmarshal(inputStream);
      }
      finally
      {
         inputStream.close();
      }
   }

   public static YoSliderboardListDefinition loadYoSliderboardListDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(YoSliderboardListDefinition.class);
         Unmarshaller unmarshaller = context.createUnmarshaller();
         return (YoSliderboardListDefinition) unmarshaller.unmarshal(inputStream);
      }
      finally
      {
         inputStream.close();
      }
   }

   public static void saveSCSGuiConfigurationDefinition(OutputStream outputStream, SCSGuiConfigurationDefinition definition) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(SCSGuiConfigurationDefinition.class);
         Marshaller marshaller = context.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
      finally
      {
         outputStream.close();
      }
   }

   public static void saveYoGraphicListDefinition(OutputStream outputStream, YoGraphicListDefinition definition) throws JAXBException, IOException
   {
      DefinitionIOTools.saveYoGraphicListDefinition(outputStream, definition);
   }

   public static void saveYoCompositePatterns(OutputStream outputStream, List<YoCompositePattern> yoCompositePatterns) throws JAXBException, IOException
   {
      saveYoCompositePatternListDefinition(outputStream, YoCompositeTools.toYoCompositePatternListDefinition(yoCompositePatterns));
   }

   public static void saveYoCompositePatternListDefinition(OutputStream outputStream, YoCompositePatternListDefinition definition)
         throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(YoCompositePatternListDefinition.class);
         Marshaller marshaller = context.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
      finally
      {
         outputStream.close();
      }
   }

   public static void saveYoEntryConfigurationDefinition(OutputStream outputStream, YoEntryConfigurationDefinition definition) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(YoEntryConfigurationDefinition.class);
         Marshaller marshaller = context.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
      finally
      {
         outputStream.close();
      }
   }

   public static void saveYoSliderboardListDefinition(OutputStream outputStream, YoSliderboardListDefinition definition) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = JAXBContext.newInstance(YoSliderboardListDefinition.class);
         Marshaller marshaller = context.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
      finally
      {
         outputStream.close();
      }
   }
}
