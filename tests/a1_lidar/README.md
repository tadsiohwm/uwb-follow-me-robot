# Host checks for A1_Lidar

Run from the repository root:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -g -Itests/a1_lidar tests/a1_lidar/check.cpp -o /tmp/check_a1_lidar
ASAN_OPTIONS=detect_leaks=0 /tmp/check_a1_lidar
```

Leak detection is disabled because the hosted environment does not allow LeakSanitizer to inspect process threads. Address and undefined-behavior instrumentation remain enabled.

The local Arduino, UART and DW1000 headers are **test doubles only**, never Arduino libraries to install. Tests cover fragmented/corrupt/max-length packets, geometry/calibration, scan boundary handling, vote and stop/recovery behavior. They do not validate real ESP32 pin routing, sensor protocol variants, radio timing, library ABI, physical identity recognition, braking or steering.
