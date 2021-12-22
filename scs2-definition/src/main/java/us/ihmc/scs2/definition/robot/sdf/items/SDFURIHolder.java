package us.ihmc.scs2.definition.robot.sdf.items;

import java.util.List;
import java.util.stream.Collectors;

public interface SDFURIHolder
{
   String getUri();

   void setUri(String uri);

   public static List<SimpleSDFURIHolder> toSimpleSDFURIHolders(List<String> uris)
   {
      return uris.stream().map(SimpleSDFURIHolder::new).collect(Collectors.toList());
   }

   public static class SimpleSDFURIHolder implements SDFURIHolder
   {
      private String uri;

      public SimpleSDFURIHolder()
      {
      }

      public SimpleSDFURIHolder(String uri)
      {
         setUri(uri);
      }

      @Override
      public String getUri()
      {
         return uri;
      }

      @Override
      public void setUri(String uri)
      {
         this.uri = uri;
      }
   }
}
