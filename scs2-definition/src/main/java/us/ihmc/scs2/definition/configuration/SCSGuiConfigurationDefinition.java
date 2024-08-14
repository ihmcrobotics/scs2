package us.ihmc.scs2.definition.configuration;

import jakarta.xml.bind.annotation.XmlRootElement;
import java.util.List;
import java.util.Objects;

@XmlRootElement(name = "SCSGuiConfiguration")
public class SCSGuiConfigurationDefinition
{
   private String name;
   private String yoGraphicsFilename;
   private String yoCompositePatternListFilename;
   private String yoEntryConfigurationFilename;
   private String yoSliderboardConfigurationFilename;
   private String yoEquationFilename;
   private String mainYoChartGroupConfigurationFilename;
   private List<String> secondaryYoChartGroupConfigurationsFilenames;

   private int bufferSize = -1;
   private int recordTickPeriod = -1;
   private int numberPrecision = -1;
   private boolean showYoSearchPanel = false;
   private boolean showOverheadPlotter = false;
   private boolean showAdvancedControls = false;
   /**
    * Whether to show the unique names of the yoVariables.
    */
   private boolean showYoVariableUniqueNames = true;
   private WindowConfigurationDefinition mainWindowConfiguration;
   private List<WindowConfigurationDefinition> secondaryWindowConfigurations;

   public void setName(String configurationName)
   {
      this.name = configurationName;
   }

   public void setYoGraphicsFilename(String yoGraphicsFilename)
   {
      this.yoGraphicsFilename = yoGraphicsFilename;
   }

   public void setYoCompositePatternListFilename(String yoCompositePatternListFilename)
   {
      this.yoCompositePatternListFilename = yoCompositePatternListFilename;
   }

   public void setYoEntryConfigurationFilename(String yoEntryConfigurationFilename)
   {
      this.yoEntryConfigurationFilename = yoEntryConfigurationFilename;
   }

   public void setYoSliderboardConfigurationFilename(String yoSliderboardConfigurationFilename)
   {
      this.yoSliderboardConfigurationFilename = yoSliderboardConfigurationFilename;
   }

   public void setYoEquationFilename(String yoEquationFilename)
   {
      this.yoEquationFilename = yoEquationFilename;
   }

   public void setMainYoChartGroupConfigurationFilename(String mainYoChartGroupConfigurationFilename)
   {
      this.mainYoChartGroupConfigurationFilename = mainYoChartGroupConfigurationFilename;
   }

   public void setSecondaryYoChartGroupConfigurationsFilenames(List<String> secondaryYoChartGroupConfigurationsFilenames)
   {
      this.secondaryYoChartGroupConfigurationsFilenames = secondaryYoChartGroupConfigurationsFilenames;
   }

   public void setBufferSize(int bufferSize)
   {
      this.bufferSize = bufferSize;
   }

   public void setRecordTickPeriod(int recordTickPeriod)
   {
      this.recordTickPeriod = recordTickPeriod;
   }

   public void setNumberPrecision(int numberPrecision)
   {
      this.numberPrecision = numberPrecision;
   }

   public void setShowYoSearchPanel(boolean showYoSearchPanel)
   {
      this.showYoSearchPanel = showYoSearchPanel;
   }

   public void setShowOverheadPlotter(boolean showOverheadPlotter)
   {
      this.showOverheadPlotter = showOverheadPlotter;
   }

   public void setShowAdvancedControls(boolean showAdvancedControls)
   {
      this.showAdvancedControls = showAdvancedControls;
   }

   public void setShowYoVariableUniqueNames(boolean showYoVariableUniqueNames)
   {
      this.showYoVariableUniqueNames = showYoVariableUniqueNames;
   }

   public void setMainWindowConfiguration(WindowConfigurationDefinition mainWindowConfiguration)
   {
      this.mainWindowConfiguration = mainWindowConfiguration;
   }

   public void setSecondaryWindowConfigurations(List<WindowConfigurationDefinition> secondaryWindowConfigurations)
   {
      this.secondaryWindowConfigurations = secondaryWindowConfigurations;
   }

   public String getName()
   {
      return name;
   }

   public String getYoGraphicsFilename()
   {
      return yoGraphicsFilename;
   }

   public String getYoCompositePatternListFilename()
   {
      return yoCompositePatternListFilename;
   }

   public String getYoEntryConfigurationFilename()
   {
      return yoEntryConfigurationFilename;
   }

   public String getYoSliderboardConfigurationFilename()
   {
      return yoSliderboardConfigurationFilename;
   }

   public String getYoEquationFilename()
   {
      return yoEquationFilename;
   }

   public String getMainYoChartGroupConfigurationFilename()
   {
      return mainYoChartGroupConfigurationFilename;
   }

   public List<String> getSecondaryYoChartGroupConfigurationsFilenames()
   {
      return secondaryYoChartGroupConfigurationsFilenames;
   }

   public int getBufferSize()
   {
      return bufferSize;
   }

   public int getRecordTickPeriod()
   {
      return recordTickPeriod;
   }

   public int getNumberPrecision()
   {
      return numberPrecision;
   }

   public boolean isShowYoSearchPanel()
   {
      return showYoSearchPanel;
   }

   public boolean isShowOverheadPlotter()
   {
      return showOverheadPlotter;
   }

   public boolean isShowAdvancedControls()
   {
      return showAdvancedControls;
   }

   public boolean isShowYoVariableUniqueNames()
   {
      return showYoVariableUniqueNames;
   }

   public WindowConfigurationDefinition getMainWindowConfiguration()
   {
      return mainWindowConfiguration;
   }

   public List<WindowConfigurationDefinition> getSecondaryWindowConfigurations()
   {
      return secondaryWindowConfigurations;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof SCSGuiConfigurationDefinition other)
      {
         if (!Objects.equals(name, other.name))
            return false;
         if (!Objects.equals(yoGraphicsFilename, other.yoGraphicsFilename))
            return false;
         if (!Objects.equals(yoCompositePatternListFilename, other.yoCompositePatternListFilename))
            return false;
         if (!Objects.equals(yoEntryConfigurationFilename, other.yoEntryConfigurationFilename))
            return false;
         if (!Objects.equals(yoSliderboardConfigurationFilename, other.yoSliderboardConfigurationFilename))
            return false;
         if (!Objects.equals(mainYoChartGroupConfigurationFilename, other.mainYoChartGroupConfigurationFilename))
            return false;
         if (!Objects.equals(secondaryYoChartGroupConfigurationsFilenames, other.secondaryYoChartGroupConfigurationsFilenames))
            return false;
         if (bufferSize != other.bufferSize)
            return false;
         if (recordTickPeriod != other.recordTickPeriod)
            return false;
         if (numberPrecision != other.numberPrecision)
            return false;
         if (showYoSearchPanel != other.showYoSearchPanel)
            return false;
         if (showOverheadPlotter != other.showOverheadPlotter)
            return false;
         if (showAdvancedControls != other.showAdvancedControls)
            return false;
         if (showYoVariableUniqueNames != other.showYoVariableUniqueNames)
            return false;
         if (!Objects.equals(mainWindowConfiguration, other.mainWindowConfiguration))
            return false;
         if (!Objects.equals(secondaryWindowConfigurations, other.secondaryWindowConfigurations))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return "name: " + name + "\nyoGraphics: " + yoGraphicsFilename + "\nyoCompositePatternList: " + yoCompositePatternListFilename
             + "\nyoEntryConfiguration: " + yoEntryConfigurationFilename + "\nyoSliderboardConfiguration: " + yoSliderboardConfigurationFilename
             + "\nmainYoChartGroupConfiguration: " + mainYoChartGroupConfigurationFilename + "\nsecondaryYoChartGroupConfigurations: "
             + secondaryYoChartGroupConfigurationsFilenames + "\nbufferSize: " + bufferSize + "\nrecordTickPeriod: " + recordTickPeriod + "\nnumberPrecision: "
             + numberPrecision + "\nshowYoSearchPanel: " + showYoSearchPanel + "\nshowOverheadPlotter: " + showOverheadPlotter + "\nshowAdvancedControls: "
             + showAdvancedControls + "\nmainWindowConfiguration: " + mainWindowConfiguration + "\nsecondaryWindowConfigurations: "
             + secondaryWindowConfigurations;
   }
}
