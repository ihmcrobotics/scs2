package us.ihmc.scs2.definition;

import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

/**
 * This class gathers the general information for a session that is being exported to file.
 */
@XmlRootElement
public class SessionInformationDefinition
{
   /** The name of the session that was running. */
   private String sessionName;
   /** The update period of the session. */
   private double sessionDTSeconds;
   /** The update period at which the data was recorded. */
   private double recordDTSeconds;

   /** The file names of the exported robots, 1 per robot. */
   private List<String> robotFileNames = new ArrayList<>();
   /** The file names of the exported terrains, 1 per terrain. */
   private List<String> terrainFileNames = new ArrayList<>();
   /** The file names of the exported robot states, 1 per robot. */
   private List<String> robotStateFileNames = new ArrayList<>();
   /** The file name of the exported yoGraphics. */
   private String graphicFileName;
   /** The file name of the exported yoRegistry structure. */
   private String registryFileName;
   /** The file name of the exported buffer data. */
   private String dataFileName;

   public SessionInformationDefinition()
   {
   }

   @XmlElement
   public void setSessionName(String sessionName)
   {
      this.sessionName = sessionName;
   }

   @XmlElement
   public void setSessionDTSeconds(double sessionDTSeconds)
   {
      this.sessionDTSeconds = sessionDTSeconds;
   }

   @XmlElement
   public void setRecordDTSeconds(double recordDTSeconds)
   {
      this.recordDTSeconds = recordDTSeconds;
   }

   @XmlElement
   public void setRobotFileNames(List<String> robotFileNames)
   {
      this.robotFileNames = robotFileNames;
   }

   @XmlElement
   public void setTerrainFileNames(List<String> terrainFileNames)
   {
      this.terrainFileNames = terrainFileNames;
   }

   @XmlElement
   public void setRobotStateFileNames(List<String> robotStateFileNames)
   {
      this.robotStateFileNames = robotStateFileNames;
   }

   @XmlElement
   public void setGraphicFileName(String graphicFileName)
   {
      this.graphicFileName = graphicFileName;
   }

   @XmlElement
   public void setRegistryFileName(String registryFileName)
   {
      this.registryFileName = registryFileName;
   }

   @XmlElement
   public void setDataFileName(String dataFileName)
   {
      this.dataFileName = dataFileName;
   }

   public String getSessionName()
   {
      return sessionName;
   }

   public double getSessionDTSeconds()
   {
      return sessionDTSeconds;
   }

   public double getRecordDTSeconds()
   {
      return recordDTSeconds;
   }

   public List<String> getRobotFileNames()
   {
      return robotFileNames;
   }

   public List<String> getTerrainFileNames()
   {
      return terrainFileNames;
   }

   public List<String> getRobotStateFileNames()
   {
      return robotStateFileNames;
   }

   public String getGraphicFileName()
   {
      return graphicFileName;
   }

   public String getRegistryFileName()
   {
      return registryFileName;
   }

   public String getDataFileName()
   {
      return dataFileName;
   }
}
