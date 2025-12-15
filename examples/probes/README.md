# Probes example (robots + probes)

This example runs a **robots + probes** system. Robots periodically broadcast log chunks; probes collect encountered logs and print them to the console. You can run it in simulation or on real Pogobots, and export the console logs to a Pandas dataframe file (`.csv` or `.feather`).

---

## Build & run in simulation

```bash
# From the base pogosim directory
./build.sh
./examples/probes/probes -c conf/probes.yaml
```

---

## Build on real robots

```bash
cd examples/probes
make clean bin                      # compile the "robots" category
# OR:
ROBOT_CATEGORY=probes make clean bin # compile the "probes" category
```

---

## Export console logs to a dataframe file

### In simulation

```bash
./examples/probes/probes -c conf/probes.yaml | tee /dev/stderr | ./examples/probes/to_dataframe.py -o test.feather
```

### On real robots

```bash
make connect TTY=/dev/ttyUSB0 | tee /dev/stderr | ./to_dataframe.py -o test.feather
```

⚠️ **Important:** if the output file already exists, new logs are **merged** into the existing data.
If you don’t want that behavior, remove the file before running the command.

---

## Alternative: save the raw logs first

If you want to archive a raw text log of the experiment:

```bash
make connect TTY=/dev/ttyUSB0 | tee your_experimental_log_file.txt
./to_dataframe.py -o test.feather your_experimental_log_file.txt
```

---

## Load the exported file in Python (Pandas)

```python
import pandas as pd

# For feather files:
df = pd.read_feather("test.feather").set_index("t_ms").sort_index()

# OR, for csv files:
df = pd.read_csv("test.csv", index_col="t_ms")
```

