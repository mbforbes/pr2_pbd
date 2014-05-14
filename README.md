# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.

## branch: mock-objects-http

### Purpose
Run user studies on "true" crowd (MTurk) with this branch.

### Description
We need to do some serious work before `mock-objects` is ready for "the crowd:"

* GUI improvements (top 5 suggestions from user study)

* Rviz &rarr; ros3djs ([website](http://robotwebtools.org/tools.html), [github](https://github.com/RobotWebTools/ros3djs))

* RQT &rarr; HTML/CSS/JS; also need roslibjs ([website](http://robotwebtools.org/tools.html), [github](https://github.com/RobotWebTools/roslibjs))

* ROS on servers (AWS?)

This was branched directly off of `mock-objects`.
