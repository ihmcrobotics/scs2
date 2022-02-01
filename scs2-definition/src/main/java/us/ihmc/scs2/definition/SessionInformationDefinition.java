package us.ihmc.scs2.definition;

import java.util.ArrayList;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement
public class SessionInformationDefinition
{
   private String sessionName;
   private double sessionDTSeconds;

   private List<String> robotFileNames = new ArrayList<>();
   private List<String> terrainFileNames = new ArrayList<>();
   private List<String> robotStateFileNames = new ArrayList<>();
   private String graphicFileName;
   private String registryFileName;
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
