# Evolutionary Algorithm — Mobile Robot Path Planning

[![CI](https://github.com/praha6248/evolutionary-path-planning/actions/workflows/ci.yml/badge.svg)](https://github.com/praha6248/evolutionary-path-planning/actions/workflows/ci.yml)
![C++17](https://img.shields.io/badge/C++-17-00599C?logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-pybind11-3776AB?logo=python&logoColor=white)

🇵🇱 [Wersja polska](README.pl.md)

A genetic algorithm that plans a path for a mobile robot on a 2D occupancy-grid map.
A path is a sequence of waypoints in Cartesian coordinates (x, y); the objective function
combines path length with a penalty for every unit of length that passes through an obstacle.

The core is written in C++17 and exposed to Python through **pybind11**, so experiments,
parameter sweeps and plots can be run from Python without recompiling.

## Evolutionary cycle

Following the classic GA scheme (`Population::next_generation`):

| Step | Function | Description |
|---|---|---|
| Population | `Population` constructor | `pop_size` individuals, each a random number (0…`max_path`) of random points on the map |
| Objective → fitness | `Population::evaluate`, `Goal_function::value` | value = length + `collision_penalty` × length inside obstacles; fitness = 1 / (1 + value) |
| Selection | `Population::selection` | roulette wheel: selection probability proportional to fitness |
| Genetic operators | `Population::genetic_operations` | one-point crossover (`crossover`) and mutation (`mutation`) |
| Replacement | `Population::replacement` | offspring replace the population; the `elite` best old individuals replace the worst offspring |

An individual stores only the intermediate waypoints. Start and goal are shared by all
individuals and are attached when the objective is evaluated.

**Mutation:** each gene (waypoint) is shifted by Gaussian noise (`mutation_sigma`) with
probability `mutation_prob`. Less often a new point is inserted (`insert_prob`) or an existing
one is removed (`delete_prob`), so the chromosome length evolves as well.

**Crossover:** each parent is cut at a different position, so children can have a different
number of points than their parents (at most `max_path`).

All parameters live in the `Parameters` struct (`inc/population.hpp`) and can be changed
from Python without recompiling.

## Project layout

```
inc/point.h, src/point.cpp                   point (xp, yp)
inc/map.hpp, src/map.cpp                     obstacle map, segment collision length
inc/goal_function.hpp, src/goal_function.cpp objective function
inc/population.hpp, src/population.cpp       population and GA cycle
src/main.cpp                                 command-line program, CSV output
bindings/module.cpp                          Python module (pybind11)
python/robot_ga/                             Python package: experiments.py, plots.py
python/example.py                            library usage example
tests/test_core.cpp                          C++ unit tests (GoogleTest)
tests/test_python.py                         Python package tests (pytest)
maps/map1.txt                                example map
plot.py                                      plots from the CLI program's CSV files
```

## Setup (once)

```bash
sudo apt install build-essential cmake python3-dev python3-venv
python3 -m venv .venv
source .venv/bin/activate              # in every new terminal
pip install pybind11 scikit-build-core numpy matplotlib
```

## Python library

Development build (the module is placed in `python/robot_ga/`):

```bash
cmake -S . -B build -Dpybind11_DIR=$(python -m pybind11 --cmakedir)
cmake --build build                    # after every C++ change
PYTHONPATH=python python python/example.py
```

Or install it as a package (`import robot_ga` then works from anywhere; reinstall after
C++ changes):

```bash
pip install .
```

Usage:

```python
import robot_ga
from robot_ga import plots

m = robot_ga.Map("maps/map1.txt")

# single run; algorithm parameters as in Parameters, e.g. mutation_prob, pop_size
r = robot_ga.run(m, generations=300, max_path=8, seed=1,
                 mutation_prob=0.15, snapshot_every=50)
print(r["best_value"], r["best_collision"], r["time_s"])

plots.plot_path(m, r["best_path"])
plots.plot_convergence(r["history"])
plots.plot_snapshots(m, r["snapshots"])
plots.animate(m, r["snapshots"]).save("results/evolution.gif", writer="pillow")

# 30 runs + statistics (best, mean, worst, std, share of collision-free paths)
series = robot_ga.run_series(m, seeds=range(30), generations=300)
print(robot_ga.summarize(series))

# step by step, directly on the Population class
pop = robot_ga.Population(m, max_path=8, seed=1,
                          params=robot_ga.make_parameters(pop_size=200))
for g in range(100):
    pop.next_generation()        # selection -> genetic operators -> replacement
print(pop.best_value(), pop.best_path())
plots.plot_population(m, pop.individuals())
```

| Item | Description |
|---|---|
| `Map(file)` | `width`, `height`, `start`, `goal`, `grid` (numpy), `is_free(x, y)`, `collision_length(...)` |
| `Parameters` / `make_parameters(**kw)` | `pop_size`, `crossover_prob`, `mutation_prob`, `mutation_sigma`, `insert_prob`, `delete_prob`, `elite`, `collision_penalty` |
| `Population(map, max_path, seed, params)` | `next_generation()`, `run(n)`, `best_path()`, `best_value()`, `mean_value()`, `best_collision()`, `individuals()`, `goal_values()`, `fitness()` |
| `goal_value(path, map, penalty)` | (value, length, collision) for any full path of shape `(n, 2)` |
| `run`, `run_series`, `summarize` | experiments and statistics |
| `plots.*` | `plot_map`, `plot_path`, `plot_convergence`, `plot_population`, `plot_snapshots`, `animate` |

## Command-line program

```bash
cmake --build build

./build/robot_ga [map] [generations] [max_points] [seed]
./build/robot_ga maps/map1.txt 300 8 1

python3 plot.py maps/map1.txt
```

The program writes `results/best_path.csv` and `results/history.csv`; `plot.py` turns them
into `results/best_path.png` (map with the path) and `results/convergence.png` (best and mean
objective value per generation). Run it from the project root.

## Tests

```bash
# C++ unit tests (GoogleTest is fetched automatically by CMake)
cmake -S . -B build -DROBOT_GA_BUILD_PYTHON=OFF -DROBOT_GA_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure

# Python package tests
pip install . pytest
pytest tests/test_python.py
```

The tests cover map parsing and validation, segment–obstacle collision length, the objective
function, determinism for a fixed seed, elitism (the best value never gets worse), validity of
individuals after many generations, and that the GA finds a collision-free detour around a wall.
GitHub Actions runs them on every push with GCC and Clang.

## Map format

```
# comment (only before the size line)
size 60 40
start 5.5 20.5
goal 55.5 20.5
....##....      <- 40 rows of 60 characters: '.' free, '#' obstacle
```

Coordinates: x to the right, y down; cell (cx, cy) covers [cx, cx+1) × [cy, cy+1).
Everything outside the map counts as an obstacle.
