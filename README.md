# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.

## branch: da

### Purpose
"da" stands for "Data Analysis;" run tests, analyze data and generate graphs with this branch.

### Description
This branch has several functions related to the analysis of collected data:

* automatically run reachability tests for users data on a test set (can take many hours for all users & tests)

* draw graphs for the results of reachability tests as well as other tests (e.g. success testing)

* processes other log files (both processed and raw) and generate graphs

This branch contains the final versions of crowd PbD graphs.
