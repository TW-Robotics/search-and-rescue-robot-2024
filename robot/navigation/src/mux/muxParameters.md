# Mux Parameters

Demultiplexing nodes are automatically instantiated based on `muxConfig.json`. For every entry in "mux", a separate node is spawned.

The mux will poll for messages and forward messages from the topic with the highest priority. The mux changes over to messages with higher priority within the cycle the change occurs in. Optionally, a number of cycles with inactivity can be specified when changing from one source to another with lower priority.

### The following parameters must be present:

- **name**: Node name of the mux
- **dt**: Time between each synchronised upadate [seconds]
- **cooldownCycles**: Number of cycles that are waited until output is switched back to a lower priority (set to 0 to disable waiting)
- **topics**: List of input topic names in descending priority
- **outTopic**: Output topic name (string)