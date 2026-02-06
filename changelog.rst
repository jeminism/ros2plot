########################
## ros2plot Changelog ##
########################

0.2.1
---
* Fix f-formatting issue with nested quotes when printing debug info for zoom selector and inspector widgets

0.2.0
---
* Added LICENSE.md
* Performance:
    * Implement MemoryBoundedDeque:
        * Guarantees automatic pruning of its own data if a memory limit is reached
        * Default memory limit 2% of available memory.
    * Improved Plot rendering framerate:
        * Implemented ScanLine approach to plot compression for fast rendering
        * Implemented grid and plot-display buffers to optimize screen print calls
    * Improved GraphInspection reactivity:
        * Implemented data chunking in GraphInspector to speed up data parsing when performing closest point matching
* Enhancements:
    * Added new widget PlotConfigurator which exposes plot visualization options:
        * Cross-column interpolation
        * High Definition Braille
        * Plotting of mean values (per-column)
    * Implement CSV integration
        * Argparse inputs:
            * '--csv' to load a specific csv
            * '--csv-default-x-key' to set a default key to use for all csv fields
    * Each Plot can now have unique x keys. Needed to support csv visualization alongside realtime topics.
        * min and max limits calculated in similar way to y axis limits.
    * Automatic visibility toggle if selected X key is incompatible with a plot due to size.
    * Implemented optional performance stats logging for debugging
        * Argparse input:
            * '--log-stats' bool flag to enable logging. default OFF
            * Logs key performance stats like frame time and data size.
        

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
