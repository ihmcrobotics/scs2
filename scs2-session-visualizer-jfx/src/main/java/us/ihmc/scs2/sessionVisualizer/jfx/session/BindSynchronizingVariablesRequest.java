package us.ihmc.scs2.sessionVisualizer.jfx.session;

public class BindSynchronizingVariablesRequest
{
   private final String addedLogName;
   private final String mainVariableName;
   private final String addedLogVariableName;

   public BindSynchronizingVariablesRequest(String addedLogName, String mainVariableName, String addedLogVariableName)
   {
      this.addedLogName = addedLogName;
      this.mainVariableName = mainVariableName;
      this.addedLogVariableName = addedLogVariableName;
   }

   public String getAddedLogName()
   {
      return addedLogName;
   }

   public String getMainVariableName()
   {
      return mainVariableName;
   }

   public String getAddedLogVariableName()
   {
      return addedLogVariableName;
   }
}
