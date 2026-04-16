# MCL Simulator

A 2D Monte Carlo Localization (MCL) simulator built with Python, Pygame, and NumPy. A robot moves around a square arena while a cloud of particles attempts to estimate its position using noisy distance-sensor readings from four cardinal directions.

## What It Does

- Simulates a robot driving around an 800×800 square arena (coordinates `-400` to `+400`) with walls on all sides.
- Casts four rays (forward, right, back, left) from the robot to measure distance to the nearest wall, with added Gaussian-style noise to mimic real sensors.
- Maintains 500 particles, each a hypothesis about where the robot might be.
- On every frame:
  1. Propagates every particle by the robot's actual motion delta plus random jitter.
  2. Weights each particle by how well its expected sensor readings match the robot's readings.
  3. Resamples particles using low-variance (systematic) resampling so high-weight particles survive and spread.
- Draws the robot (red), particles (white dots), and the current position estimate (blue dot) in real time.

## Controls

| Key | Action |
| --- | --- |
| `W` | Move forward |
| `A` | Rotate counter-clockwise |
| `D` | Rotate clockwise |

Forward speed is 50 units/sec, rotation speed is 90°/sec.

## Project Structure

```
MCL-simulator/
├── main.py         # Main loop: rendering, input, MCL update step
├── mcl.py          # MCL-related helpers
├── robot.py        # Robot class (pose, motion deltas)
├── run.bat         # Windows launcher
├── pyproject.toml  # Poetry project config
└── poetry.lock
```

## Requirements

- Python 3.x
- [Poetry](https://python-poetry.org/) (recommended), or `pip`
- Dependencies: `pygame`, `numpy`

## Installation

Clone the repo:

```bash
git clone https://github.com/dhruviyengar/MCL-simulator.git
cd MCL-simulator
```

Using Poetry:

```bash
poetry install
```

Or with pip:

```bash
pip install pygame numpy
```

## Running

With Poetry:

```bash
poetry run python main.py
```

Plain Python:

```bash
python main.py
```

On Windows you can also double-click `run.bat`.

## How It Works (Quick Tour)

**Sensor model.** `ray_collision(x, y, heading, noise=False)` in `main.py` computes the distance from `(x, y)` along a given heading to the nearest wall by intersecting the ray with the four wall lines and taking the closest hit. When `noise=True`, a small uniform perturbation is added.

**Motion model.** Each particle is shifted by the robot's `deltaX`/`deltaY` since the previous frame, plus independent uniform noise, so the particle cloud expands a bit every tick.

**Measurement update.** For each particle, we compute expected forward/right/back/left ray distances and compare them to the robot's real readings. Particle weight is the product of four Gaussian likelihoods (one per direction).

**Resampling.** Low-variance resampling walks through the weight space with a single random offset and evenly spaced picks, producing a new particle set where survival probability is proportional to weight. The mean of the resampled particles is drawn in blue as the current position estimate.

## Known Quirks / Notes

- The likelihood `sigma` is intentionally very large (`10000`) to keep the filter forgiving while iterating; tune it down for sharper convergence.
- The sensor direction at `+90°` in `ray_collision` is labeled "right" in the code but, depending on your coordinate convention, you may want to double-check orientation.
- Particles that wander outside the arena are randomly respawned inside.

## License

No license specified. Add one if you plan to share or reuse the code.
