find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(
  test_smart_pad.py
  TARGET test_smart_pad
  TIMEOUT 30
)

add_launch_test(
  test_padflie.py
  TARGET test_padflie
  TIMEOUT 60
)
