# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.

## branch: mock-objects-pr2

### Purpose
Collect object positions (before user study) with this branch.

### Description
This branch was used to record object positions on the PR2 in order to get baseline positions and rotations of real-world objects on the table. It prints the coordinates of objects it scans (and I think it can also create 'mock' boxes in the UI of provided boxes, allowing you to compare virtual vs real object positions).

This isn't actively used during user studies; it was merely used to collect prerequisite data for running the studies.
