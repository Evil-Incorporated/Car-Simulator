# Car-Simulator
## Installation:

```clojure
pip install -e.
```

## Executing the environment:

```clojure
import car_simulator
import gym
envs = gym.make('Car-Simulator-v0')
envs.reset()
```

To run the environment without display:
```clojure
envs = gym.make('Car-Simulator-v0', offScreen=True)
```
