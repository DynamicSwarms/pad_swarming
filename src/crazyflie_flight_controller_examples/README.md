# Crazyflie flight-controller examples

Run the simulated scalability example:

```bash
ros2 launch crazyflie_flight_controller_examples scaling.launch.py count:=20
```

Arguments: `count` (20), `spacing` (0.35 m), `seed` (42), `area` (5.0 m),
`speed` (0.6 m/s), and `visualize` (`true`). For example:

```bash
ros2 launch crazyflie_flight_controller_examples scaling.launch.py \
  count:=50 area:=7.0 speed:=0.5 visualize:=false
```
