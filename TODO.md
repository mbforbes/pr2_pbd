# TODO

## Benchmark
1. confirm w/ maya re: object grounding (which props? referred or try to infer intent?)
2. implement answer to 1.
3. hook up referring phrase generation (may be part of 2., may not)
4. close & open should always be recorded in program, I think... perhaps this means core and post-check fails shouldn't be considered, but pre-check should (as this rules out accidents)

## Test 1
- fix property calculation (at least one of leftmost, biggets, etc. should always be true)
- implement screenshotting & logging (simple)
- plan tests
- run tests

Extra credit:

- 'Middle' is a very natural thing to want to say in many situations. We're probably not doing it because it's harder to do automatically. However, it is occasionally very useful (e.g. with three similar objects).

## Original list
0. [Parser is prereq] Store / resolve obj referring phrases.
0. [Parser is prereq] 'Turn hand' functionality
0. [Parser is prereq] Movement parameters (how far)
0. [Parser is prereq] Record objects command.
0. Any addl. meta-programming? May arise after using system more. (Then -> parser.)
0. Fix gaze if still broken?
0. Both hands. This is a huge thing (to implement), not to be taken lightly. (Then -> parser.)
0. all movement w/ `pr2_object_manipulation` or MoveIt! for collision avoidance, modeling grasped object, moving faster than glacier speed, etc. Maybe want cartesian control (e.g. without collision detection) still as override??
