# Making a Release of Simulation Construction Set 2
1. Bump the version in `group.gradle.properties`
2. Commit only `group.gradle.properties` with a message in the format: "`:bookmark: <version>`"
3. Create a tag with the version name
4. Push the release commit and tag
5. Ensure publishing credentials for IHMC robotlabfiles are set in your `~/.gradle/gradle.properties` file.
6. Publish using `gradle compositePublish -PpublishUrl=robotlabfiles`
7. Build a Debian .deb installer using `cd docker/debian; ./buildDebianInstaller.sh`
8. Build the Windows .msi installer on a Windows machine that has WiX Toolset 3.x and JDK 17 (providing `jpackage` and `jlink`) available. Run from the repository root: `gradle :scs2-session-visualizer-jfx:buildWindowsPackagesJlink`. The MSI is written to `scs2-session-visualizer-jfx/deployment/windows/msi-jlink/SCS2SessionVisualizer-<version>.msi`. (The Stage 1 task `buildWindowsPackages` produces a slightly larger MSI without the jlink-trimmed runtime.)
9. Create a release on GitHub documenting the changes (following the format of existing releases)
10. Upload the .deb (located in `scs2-session-visualizer-jfx/deployment/debian`) and the .msi (located in `scs2-session-visualizer-jfx/deployment/windows/msi-jlink`) created previously to the new GitHub release
11. Announce the release to whoever may be interested
