package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.SessionDataFilterParameters;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;

// TODO This class is overkill, I couldn't find a place where to put the preferred filters for exporting data
public class SessionDataPreferenceManager implements Manager
{
   private final Map<String, SessionDataFilterParameters> filterMap = new LinkedHashMap<>();

   public SessionDataPreferenceManager(Messager messager, SessionVisualizerTopics topics)
   {
      messager.addTopicListener(topics.getSessionDataFilterParametersAddRequest(), m ->
      {
         if (m.getName() == null)
            LogTools.error("Session data filter name cannot be null");
         else
            filterMap.put(m.getName(), m);
      });
   }

   public Map<String, SessionDataFilterParameters> getFilterMap()
   {
      return filterMap;
   }

   @Override
   public void startSession(Session session)
   {
   }

   @Override
   public boolean isSessionLoaded()
   {
      return true;
   }

   @Override
   public void stopSession()
   {
   }
}
