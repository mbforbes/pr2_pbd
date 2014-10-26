# Hands-Free PbD

The work in this repository is under publication review. **Please contact me for citations and usage.**

## Quick-start guide
This guide is intended for those familiar with PbD, and who have the [Hands-Free PbD Parser](https://github.com/mbforbes/hfpbd-parser) on their computer. If that's not you, please proceed to the "Running" section below.

```bash
# FIRST: EDIT pr2_pbd_interaction/src/util.py and change EXP_DIR to
# a directory that's not mine. If you're using this, I would love a
# PR that fixes this; it's simple, I just don't have time to test it.

# On the robot
$ roslaunch pr2_pbd_interaction pbd_backend.launch

# On the desktop
$ roslaunch pr2_pbd pbd_frontend.launch

# Also on the desktop, launch the Hands-Free PbD Parser.
$ python parser/web/web_interface.py
```

## Running

### Robot code
Please see the [main PbD repository](https://github.com/PR2/pr2_pbd) for general instructions on setting up and running PbD. The process is the same for this version of the code, with one additional requirement:

### Parser code
You must also run the [Hands-Free PbD Parser](https://github.com/mbforbes/hfpbd-parser). See that page for instructions, including how to modify and extend the grammar, how to set-up voice versus keyboard input, and how to run the various supported frontends, including the default vocabulary you need to use to interact with the system.

### Cool feature: PR2 records everything!
```bash
# NOTE: EDIT pr2_pbd_interaction/src/util.py and change EXP_DIR to
# a directory that's not mine. If you're using this, I would love a
# PR that fixes this; it's simple, I just don't have time to test it.
```

With that said, the system automatically saves a rosbag (`.bag`) in a convenient directory structure in every run that contains:

- Snapshot pictures of the first-person view of what the robot sees from the Kinect. It takes snapshots anytime a system event happens. Great for going through actions to figure out what happened!
- All commands
- All robot speech
- All grounding queries
- All object descriptions


## Language input
A general note for the language input: speech is supported, but we have not fine-tuned the system to be demo-able with it.

Here are the steps we've taken so far:

- We've combined certain word pairs like `left-hand` and `in-front-of` in the grammar for better speech recognition (notably at the expense of typing input)
- We've implemented a dedicated mode in the parser that will generate a full corpus of training sentences that can be used to generate settings files for speech recognition programs like Pocketsphinx
- We've tested using standard speech input and the system functions

With that said, we have not yet worked to make the system demo-able and rebust enough to handle naive users. While we use a vastly restricted vocabulary, its domain is large enough, and its impact on the system great enough (i.e. complete control), that getting reliable speech input without a dialog manager is a nontrivial task.
