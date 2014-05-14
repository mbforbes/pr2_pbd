# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.

## branch: interface-video

### Purpose
Record the video (screen recording) of the user interface with this branch.

### Description
This branch was a tiny offshoot off of normal development where I hardcoded in the size/position/rotation of a new object (lavamoss) *not* used in the user study. I then used this object to record a video of the user interface as a tutorial for a worker to watch.

I used a different object so that workers who viewed the video would not get extra information about how to complete their task.

In case it's not clear from the above, the code is quite old (pre-active sampling) and should only be used for user interface video-purposes.
