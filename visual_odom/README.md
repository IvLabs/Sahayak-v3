# Visual Odometry
## Running
* Install rowan `pip install rowan`
* Launch `launch_all.launch` before, this publishes all the prerequisite topics.
* Run the python file of your choosing and control the bot.

## Attempts:
The best attempt is `pnp.py`, uses PnP with Ransac to perform 3D-2D motion estimation.

Results: [Drive Link](https://drive.google.com/file/d/1cUCRjERNW7lkDszR3cxhvb3hW1r-Ev1N/view?usp=sharing)

It messes up sometimes, but on the whole, in an area with great features, it works well.