# pr2_pbd
Using [Maya Cakmak's PbD system](https://github.com/PR2/pr2_pbd) as a base, my fork contains branches for different projects, as well as branches for different aspects within each project.

This means there are lots of branches, and it can be hard to tell what is what.

Git's branching system doesn't support branch descriptions, and branch names need to be concise, so the this `README.md` file in the root of the repository will contain information about the branch it's in.

## branch: profile
### Purpose
See what's used (and what's not) in PbD, as well as what's slowing us down (if anything).

### Description
There is a lot of code in PbD that is
- legacy (deprecated and unsupported)
- undocumented
- confusing (variable names imply incorrect object types)
- unorganized (public / private / static / class functions mixed around)
- slow (marker movement w/ IK calls)

I think profiling is the first step just to see what can be cleaned up and perhaps get a vauge handle to what's taking the most time.
