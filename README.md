# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.


This branch (`groovy-devel`) should just be a mirror of the groovy-devel branch of the main PbD code ([PR2/pr2_pbd:groovy-devel](https://github.com/PR2/pr2_pbd)). I will list below information about all of the branches that branch off of this (that is, the projects that are based off of the groovy PbD code).

I'll try to keep the branch descriptions up to date below in this readme, but most of the documentation will likely be from the main documentation effort I did on May 14, 2014.

I'll try even harder to keep the branch descriptions up to date in the README.md file of each branch. Check there as the authority for documentation.

Branches are listed alphabetically below by project. At the bottom are the 'default' branches (that came with the fork).

# Projects

## PbD + NLP

### branch: hands-free
#### Purpose
Language-only PbD.

#### Description
WIP.


### branch: nlplite
#### Purpose
Less constrained language &rarr; PbD commands.

#### Description
Joint work with the UW SPF folks (Yoav + Kenton).


### branch: test-grasping
#### Purpose
This branch is for trying out the automatic grasping capabilities built into the PR2 Object Manipulation stack (pr2_interactive_manipulation).

#### Description
I'm testing out the built-in functionality by trying to run demos and then using API calls.



## PbD &rarr; hydro, catkin

### repo: [vovakkk/pr2_pbd:hydro](https://github.com/vovakkk/pr2_pbd/tree/hydro)
#### Purpose
Get PbD migrated to hydro & catkin.

#### Description
Vladimir did a near complete rewrite of the PbD code, and took the opportunity to bring about several new features:

* Use MoveIt! as the PR2 controller
* Javascript / HTML / CSS button UI using rosbridge (no RQT)
* planned: Javascript / WebGL UI using ros3djs (no Rviz)
* YAML action representation
* Richer action representation (action reference, trajectories)
* Various other library migrations because of catkin and/or Hydro, usually to a newer and better library



## PbD++

### branch: profile
#### Purpose
See what's used (and what's not) in PbD, as well as what's slowing us down (if anything).

#### Description
There is a lot of code in PbD that is
- legacy (deprecated and unsupported)
- undocumented
- confusing (variable names imply incorrect object types)
- unorganized (public / private / static / class functions mixed around)
- slow (marker movement w/ IK calls)

I think profiling is the first step just to see what can be cleaned up and perhaps get a vauge handle to what's taking the most time.



## Crowd PbD

### branch: da
#### Purpose
"da" stands for "Data Analysis;" run tests, analyze data and generate graphs with this branch.

#### Description
This branch has several functions related to the analysis of collected data:

* automatically run reachability tests for users data on a test set (can take many hours for all users & tests)

* draw graphs for the results of reachability tests as well as other tests (e.g. success testing)

* processes other log files (both processed and raw) and generate graphs

This branch contains the final versions of crowd PbD graphs.


### branch: generate-objects
#### Purpose
Generate new test data (for users to fix, or to run evaluation) with this branch.

#### Description
"Test data" means the automatic sampling of object positions / rotations, using a given seed and object sizes / numbers, for varied (but specified) numbers of unreachable poses.

This code, once launched via a click on the GUI, will continue sampling data until the specified quantity is fulfilled. (Look in launch file or code for these quantities.)

The data is a 'fresh set' of experiment data, and as such can be used either as a worker's starting point to collect data, or left untouched and used as test data (reachability and/or success).


### branch: interface-video
#### Purpose
Record the video (screen recording) of the user interface with this branch.

#### Description
This branch was a tiny offshoot off of normal development where I hardcoded in the size/position/rotation of a new object (lavamoss) *not* used in the user study. I then used this object to record a video of the user interface as a tutorial for a worker to watch.

I used a different object so that workers who viewed the video would not get extra information about how to complete their task.

In case it's not clear from the above, the code is quite old (pre-active sampling) and should only be used for user interface video-purposes.


### branch: mock-objects
#### Purpose
Run user studies with this branch.

#### Description
This is the branch on which the Crowd PbD project is based. It serves as the "main branch" off of which most other code is branched. User studies may be run using this branch.


### branch: mock-objects-http
#### Purpose
Run user studies on "true" crowd (MTurk) with this branch.

#### Description
We need to do some serious work before `mock-objects` is ready for "the crowd:"

* GUI improvements (top 5 suggestions from user study)

* Rviz &rarr; ros3djs ([website](http://robotwebtools.org/tools.html), [github](https://github.com/RobotWebTools/ros3djs))

* RQT &rarr; HTML/CSS/JS; also need roslibjs ([website](http://robotwebtools.org/tools.html), [github](https://github.com/RobotWebTools/roslibjs))

* ROS on servers (AWS?)

This was branched directly off of `mock-objects`.


### branch: mock-objects-pr2
#### Purpose
Collect object positions (before user study) with this branch.

#### Description
This branch was used to record object positions on the PR2 in order to get baseline positions and rotations of real-world objects on the table. It prints the coordinates of objects it scans (and I think it can also create 'mock' boxes in the UI of provided boxes, allowing you to compare virtual vs real object positions).

This isn't actively used during user studies; it was merely used to collect prerequisite data for running the studies.


### branch: success-testing
#### Purpose
Test user-provided fixes on real test data on the real robot (desktop version, with GUI) on this branch.

#### Description
This branch contains the score functions as well as a full GUI for loading up test scenarios, automatically scoring them based on different score functions, and loading the action. It is the desktop component of the success testing.

As a prerequisite, you must have done reachability testing (all worker actions vs all test scenarios). This is because doing so is expensive (dozens of hours) and so should be done offline and beforehand. Because of this requirement, we can quickly filter out unreachable demonstrations and score only the reachable ones. (This data is a log file, generated by branch `da`).


### branch: success-testing-pr2
#### Purpose
Test user-provided fixes on real test data on the real robot (PR2 version) with this branch.

#### Description
This branch contains the PR2-side of doing success testing. This means that when a test scenario is loaded, it will send to the GUI 'mocked' (white silhouette) boxes for the locations of the objects in the test scenario, as well as boxes (green) for the 'true' objects that it scans and recognizes.

When doing success testing, the goal is to match the 'real world' object as closely as possible with the mocked version that is rendered (as this is the test case).

For more info on success testing (score functions, prerequisites) see the desktop version of this branch, `success-testing`.


## ICRA 2014

## branch: priya
### Purpose
Code to run a demo of Mike's ICRA 2014 paper on the PR2 (instead of Gambit).

### Description
This runs a loop where the robot asks for colored blocks in an 8 x 8 grid and sets them down on a plane in front of it. It is based off of [Priya Rao's code](https://github.com/priyarao27/pr2_pbd/) (she did the majority of the work). Implementation notes are [here](https://github.com/mbforbes/pr2_pbd/wiki/Press-Release:-Plans).

This code is a demo for the press release for the ICRA 2014 work, which actually doesn't run PbD at all, but rather Mike's crowdsourcing framework. However, we had to switch the demo from running on the Gambit to the PR2 because the Gambit is out of commission, and the PbD framework provided an easy code foundation on which to do this.


# Defaults

### branch: groovy-devel
#### Purpose
This branch contains the 'main' PbD code, off of which others fork and build. It's in "development." As far as I know, this is never built / realeased into binaries.

#### Description
All project-agnostic improvements and bugfixes should go here.

### branch: groovy
#### Purpose
As far as I know, this branch is the release that is built into (debian) binaries for ROS distribution with apt-get.

### Description
I think that this branch is only for marking release points in groovy-devel. I could be wrong, though.

### branch: pr2_pbd_speech_recognition
#### Purpose
As far as I know, this branch is a legacy branch that marks a point on the groovy-devel line that is pre-groovy (i.e. before the branch marked 'groovy'), and serves no purpose.

### Description
Just a legacy checkpoint, looks like.
