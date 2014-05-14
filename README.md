# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.

## branch: generate-objects

### Purpose
Generate new test data (for users to fix, or to run evaluation) with this branch.

### Description
"Test data" means the automatic sampling of object positions / rotations, using a given seed and object sizes / numbers, for varied (but specified) numbers of unreachable poses.

This code, once launched via a click on the GUI, will continue sampling data until the specified quantity is fulfilled. (Look in launch file or code for these quantities.)

The data is a 'fresh set' of experiment data, and as such can be used either as a worker's starting point to collect data, or left untouched and used as test data (reachability and/or success).
