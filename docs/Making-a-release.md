# Making a Release of Simulation Construction Set 2
1. Bump the version in `group.gradle.properties`
2. Commit only `group.gradle.properties` with a message in the format: "`:bookmark: <version>`"
3. Create a tag with the version name
4. Push the release commit and tag. Pushing a tag matching `17-*` automatically triggers the `Release`
   GitHub Actions workflow (`.github/workflows/release.yml`): it builds the Debian `.deb`, Windows `.msi`,
   and macOS `.dmg` installers in parallel on `ubuntu-latest`/`windows-latest`/`macos-latest` runners, then
   creates a GitHub Release for that tag with all three attached and auto-generated release notes. Check
   the Actions tab to watch its progress; it does not require anyone to run a Windows or Mac build locally.
5. Ensure publishing credentials for IHMC robotlabfiles are set in your `~/.gradle/gradle.properties` file.
6. Publish using `gradle compositePublish -PpublishUrl=robotlabfiles`. This is a separate, still-manual step
   - the `Release` workflow only builds/uploads the installers, it does not publish Maven artifacts.
7. Once the `Release` workflow finishes, edit the release's notes on GitHub to match the format of existing
   releases, if the auto-generated notes aren't sufficient on their own.
8. Announce the release to whoever may be interested.

## Note: no macOS build for the ZED SDK
The ZED SDK has no macOS build, so ZED video log data is unsupported on macOS and is skipped at runtime
(see `MultiVideoDataReader`).

## Rebuilding without cutting a new release
- Re-run the `Release` workflow against an existing tag without pushing a new one: Actions tab → `Release`
  → `Run workflow`, and enter the tag name. This rebuilds all three installers and re-uploads them onto
  that tag's existing release.
- Or build a single platform's installer locally, the way it was done before this workflow existed:
  - **Linux**: `cd docker/debian; ./buildDebianInstaller.sh` (runs in Docker, no Linux-specific host
    requirements). Output: `scs2-session-visualizer-jfx/deployment/debian/scs2-<version>.deb`
  - **Windows**: on a Windows machine with WiX Toolset 3.x and JDK 17 (providing `jpackage`/`jlink`),
    from the repository root: `gradle :scs2-session-visualizer-jfx:buildWindowsPackagesJlink`. Output:
    `scs2-session-visualizer-jfx/deployment/windows/msi-jlink/SessionVisualizer-<version>.msi`. (The
    Stage 1 task `buildWindowsPackages` produces a slightly larger MSI without the jlink-trimmed runtime.)
  - **macOS**: on a Mac with JDK 17 (providing `jpackage`/`jlink`), from the repository root:
    `gradle :scs2-session-visualizer-jfx:buildMacPackagesJlink`. Output:
    `scs2-session-visualizer-jfx/deployment/mac/dmg-jlink/SessionVisualizer-<version>.dmg`. (The Stage 1
    task `buildMacPackages` produces a slightly larger DMG without the jlink-trimmed runtime.)
