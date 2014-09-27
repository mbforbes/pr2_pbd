# TODO

## Bugs
Fix **only if encountered:**

- [parser] top sentences specifies one object, all descriptions match, but command returns with different object

- [parser] entering 'asdf' with a ton of sentences with 0.000085 score and a ton of commands with score 0.0090 (and lang_score 0.0065 (?)) returns a command!! WTF?!?!
	- possible fix: normalize to 100 instead of 1, specify that if all commands within X percent (10% ?) of the top command's score (not total score) are equal and need to be clarified. This should (?) be score-independent.

- referring phrase generation is wrong:
	- print dictionary
	- [+ parser] parse & describe as service calls

- close & open aren't recorded when they should have been
	- possible fix: core and post-check fails shouldn't be considered, but pre-check should (as this rules out accidents)

- property calculation (at least one of leftmost, biggets, etc. should always be true)

## Optimizations
Implement only if making production (user-test) ready or needed for tests:
- [+ Parser] Movement parameters (how far) (+ default)
- [+ Parser] Record objects command.
- [+ Parser] Edit / programming modes
- [+ Parser] Both hands. This is a huge thing (to implement), not to be taken lightly. (Then -> parser.)
- All movement w/ `pr2_object_manipulation` or MoveIt! for collision avoidance, modeling grasped object, moving faster than glacier speed, etc. Maybe want cartesian control (e.g. without collision detection) still as override??
- [+ Parser] Make speech recognition more robust
