# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.

## branch: success-testing-pr2

### Purpose
Test user-provided fixes on real test data on the real robot (PR2 version) with this branch.

### Description
This branch contains the PR2-side of doing success testing. This means that when a test scenario is loaded, it will send to the GUI 'mocked' (white silhouette) boxes for the locations of the objects in the test scenario, as well as boxes (green) for the 'true' objects that it scans and recognizes.

When doing success testing, the goal is to match the 'real world' object as closely as possible with the mocked version that is rendered (as this is the test case).

For more info on success testing (score functions, prerequisites) see the desktop version of this branch, `success-testing`.
