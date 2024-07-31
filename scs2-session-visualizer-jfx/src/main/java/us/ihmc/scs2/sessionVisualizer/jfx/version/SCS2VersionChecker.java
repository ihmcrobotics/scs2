package us.ihmc.scs2.sessionVisualizer.jfx.version;

import com.google.gson.Gson;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;

import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;

/**
 * This class is used to check if the current version of SCS2 is the latest one.
 * <p>
 * The current version is retrieved from the MANIFEST.MF file.
 * </p>
 * <p>
 * The latest version is retrieved from the GitHub API.
 * </p>
 */
public class SCS2VersionChecker
{
   /**
    * The current version of SCS2.
    */
   private static String CURRENT_BASE_VERSION;
   /**
    * The latest release of SCS2 with its version and URL.
    */
   private static Release LATEST_RELEASE;
   /**
    * The latest version of SCS2.
    */
   private static String LATEST_BASE_VERSION;

   /**
    * The URL to the GitHub API to retrieve the latest release of SCS2.
    */
   private static final URL REPOSITORY_API_URL;

   public static final URL REPOSITORY_URL;
   public static final URL DOWNLOAD_URL;

   static
   {
      try
      {
         REPOSITORY_API_URL = new URL("https://api.github.com/repos/ihmcrobotics/simulation-construction-set-2/releases/latest");
         REPOSITORY_URL = new URL("https://www.github.com/ihmcrobotics/simulation-construction-set-2");
         DOWNLOAD_URL = new URL("https://github.com/ihmcrobotics/simulation-construction-set-2/releases/latest");
      }
      catch (MalformedURLException e)
      {
         throw new RuntimeException(e);
      }
   }

   /**
    * Returns the current version of SCS2.
    * <p>
    * The current version is retrieved from the MANIFEST.MF file.
    * </p>
    *
    * @return the current version of SCS2.
    */
   public static String getCurrentBaseVersion()
   {
      if (CURRENT_BASE_VERSION == null)
      {
         String version = SCS2VersionChecker.class.getPackage().getImplementationVersion();
         CURRENT_BASE_VERSION = version == null ? "[source-code-version]" : toBaseVersion(version);
      }
      return CURRENT_BASE_VERSION;
   }

   /**
    * Returns the latest release of SCS2 with its version and URL.
    * <p>
    * The latest release is retrieved from the GitHub API.
    * </p>
    *
    * @return the latest release of SCS2.
    */
   public static Release getLatestRelease()
   {
      if (LATEST_RELEASE == null)
      {
         OkHttpClient client = new OkHttpClient();
         Request request = new Request.Builder().url(REPOSITORY_API_URL).build();

         try (Response response = client.newCall(request).execute())
         {
            if (!response.isSuccessful())
               throw new IOException("Unexpected code " + response);

            Gson gson = new Gson();
            LATEST_RELEASE = gson.fromJson(response.body().string(), Release.class);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
      return LATEST_RELEASE;
   }

   /**
    * Returns the latest version of SCS2.
    * <p>
    * The latest version is retrieved from the GitHub API.
    * </p>
    *
    * @return the latest version of SCS2.
    */
   public static String getLatestBaseVersion()
   {
      if (LATEST_BASE_VERSION == null)
      {
         LATEST_BASE_VERSION = toBaseVersion(getLatestRelease().tag_name);
      }
      return LATEST_BASE_VERSION;
   }

   public static String getLatestReleaseURL()
   {
      return getLatestRelease().html_url;
   }

   /**
    * Returns whether the current version of SCS2 is the latest one.
    * <p>
    * The current version is retrieved from the MANIFEST.MF file.
    * </p>
    * <p>
    * The latest version is retrieved from the GitHub API.
    * </p>
    *
    * @return {@code true} if the current version of SCS2 is the latest one, {@code false} otherwise.
    */
   public static boolean isLatestRelease()
   {
      return getCurrentBaseVersion().equals(getLatestBaseVersion());
   }

   public static class Release
   {
      private String tag_name;
      private String html_url;
   }

   /**
    * Converts a version string to its base version.
    * <p>
    * For example, {@code "17-0.0.1"} is converted to {@code "0.0.1"}.
    * </p>
    *
    * @param version the version to convert.
    * @return the base version.
    */
   private static String toBaseVersion(String version)
   {
      // Later replace these with regex to be more robust.
      return version.replace("17-", "").replace("-java-17", "").trim();
   }
}
