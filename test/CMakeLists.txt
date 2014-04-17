set(TEST_CASES voltage_reading cannonball vehicle_accelerometer)

foreach(test ${TEST_CASES})
	add_executable(test_${test} test_${test}.cpp)
	add_test(test_${test} test_${test})
endforeach(test)

