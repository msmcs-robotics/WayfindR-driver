Tested on Remote System
The localization was tested on the remote system (192.168.0.7) and confirmed working:
LiDAR connected (Slamtec C1M1RP)
Map loaded successfully (212x144 @ 0.05m/cell)
AMCL node activated via lifecycle manager
Ready for initial pose estimate via RViz
How to Use
On the remote system's physical display:

~/start_localization.sh
Then in RViz, click "2D Pose Estimate" and click/drag on the map to set where you think the robot is. The particle cloud will converge as AMCL matches LiDAR scans to the map