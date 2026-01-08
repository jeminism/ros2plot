########################
## ros2plot Changelog ##
########################

0.2.0
---
* Performance:
    * Implement MemoryBoundedDeque:
        * Guarantees automatic pruning of its own data if a memory limit is reached
        * Default memory limit 2% of available memory.

0.1.2
---
* Fix:
    * Fix issue where ghost points may appear when using the inspector due to new points in the data set which are not yet rendered when paused
    * Fix issue where artefacts from the dropdown list in selector remain after closing it
    * Fix incorrect import path is ros package init script
    * Fix issue where all plots would be re-added and become visible if receiving a subscription command to '/'
* Enhancements:
    * Scrollable zoom window. 
        * Rework implementation for zoom_lock flag, 
        * Allows the window size to be locked as per the zoom selector widget
        * Graph rendering is still occuring the background, allowing the plot to be reactive to the window position
    * Inspector now supports multi-point parsing.
        * Re-worked y-value determination when parsing for a particular x value.
        * All y-points which match the x-pixel location will now be displayed. 
        * This also fixes issue of improper display of y points when using non-linear x-axis fields.
    

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
