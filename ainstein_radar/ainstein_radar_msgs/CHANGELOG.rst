^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ainstein_radar_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.1 (2020-02-11)
------------------

3.0.0 (2020-02-06)
------------------

2.0.2 (2019-11-19)
------------------

2.0.1 (2019-11-12)
------------------

2.0.0 (2019-11-12)
------------------
* Update RadarInfo msg, refactor for K79 and add T79
  Updated the RadarInfo message slightly. Also refactored the K79
  interface class to publish the RadarInfo message.
  Added publishing the RadarInfo message for T79 as well, needs testing.
* Update RadarInfo message and fix dependencies
  Updated the RadarInfo message with new fields fully describing the data
  and added comments to explain them.
  Also fixed a small dependency in the RViz plugins package for display
  of the RadarInfo messages, however this plugin is not complete anyway.
* Contributors: Nick Rotella

1.1.0 (2019-10-29)
------------------
* Minor fixes to package XML formatting
  Fixed the package XML file formatting and added missing content to
  conform to the suggested style guidelines.
* Contributors: Nick Rotella

1.0.3 (2019-10-03)
------------------

1.0.2 (2019-09-25)
------------------

1.0.1 (2019-09-24)
------------------
* Minor, remove unnecessary install target for radar_msgs
* Merge branch 'master' of https://github.com/AinsteinAI/ainstein_radar
* Add ainstein_radar_msgs/RadarInfo msg definition
  Added a new message type to ainstein_radar_msgs to store information
  about a radar sensor's properties, configuration, etc. Currently
  contains physical limits for range, speed and angles.
* Fix radar stamped msg, add nearest target filter
  Fixed the RadarTargetStamped message to use the unstamped RadarTarget
  message rather than duplicating fields.
  Added a nearest target filter which extracts the nearest target (by
  range) within set min/max range bounds and optionally low-pass filters
  it before publishing as both a RadarTargetStamped and as an array with
  one message (called "tracked").  Will remove the array published after
  implementing a proper tracked target filter.
* Refactor radar message types, other ainstein_radar subpkgs WILL NOT BUILD
  Refactored the message types defined for radars, breaking all other subpkgs
  in the ainstein_radar metapkg. Now refactoring all other pkgs to use the
  new message definitions.
* Migrate old radar_sensor_msgs pkg to new ainstein_radar_msgs subpkg
* Contributors: Nick Rotella
