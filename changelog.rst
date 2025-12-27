########################
## ros2plot Changelog ##
########################

0.1.1
---
* Added Changelog, README documentation
* Setup build system compatibility:
    * Added setup.py, setup.cfg, requirements.txt for pip and colcon installations
* Refactored import structure to use relative instead of absolute paths.
* Implement Argparse to parse subscription inputs during run-time
* Fixes:
    * Fix issue where pause doesnt actually pause the rendering
    * Fix issue where plot rendering will always start at -1,-1
    * Fix issue where plots will become invisible after terminal resizing

0.1.0
---
* Initial version
