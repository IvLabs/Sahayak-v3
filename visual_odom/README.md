# Visual Odometry
## Running
* Install rowan `pip install rowan`
* Launch `launch_all.launch` before, this publishes all the prerequisite topics.
* Run the python file of your choosing and control the bot.
* launch the maps with/without running the optical flow node(RTAB has its own VO node and Hector makes grid maps with 2d lidar only).

## Attempts:
The best attempt is `optical_flow.py`, uses PnP with Ransac to perform 3D-2D motion estimation.

Results For Visual Odometry node: [Drive Link](https://drive.google.com/file/d/1cUCRjERNW7lkDszR3cxhvb3hW1r-Ev1N/view?usp=sharing)

Results For RTAB Map:[Drive Link](https://drive.google.com/file/d/1q-6FVi4KLn8yQ2v2SscourI8IZMEwSHz/view?usp=sharing)



It messes up sometimes, but on the whole, in an area with great features, it works well.

