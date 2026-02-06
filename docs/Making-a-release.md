# Making a Release of Simulation Construction Set 2
1. Bump the version in `group.gradle.properties`
2. Commit only `group.gradle.properties` with a message in the format: "`:bookmark: <version>`"
3. Create a tag with the version name
4. Push the release commit and tag
6. Setup Publishing according to [Confluence](https://ihmcrobotics.atlassian.net/wiki/spaces/PUBLIC/pages/14844236/Releasing+a+Standalone+Library).
6. Publish using `gradle compositePublish -PpublishUrl=ihmcRelease`
7. Build a Debian .deb installer using `cd docker/debian; ./robotlabfiles.sh`
8. Create a release on GitHub documenting the changes (following the format of existing releases)
9. Upload the .deb (located in `scs2-session-visualizer-jfx/deployment/debian`) created previously to the new GitHub release
10. Announce the release to whoever may be interested