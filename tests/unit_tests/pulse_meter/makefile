run: main
	./test_pulse_meter
main:
	g++ -g -DESPHOME_LOG_LEVEL=3 -I. -Iesphome/components/pulse_meter test_pulse_meter.cpp ../../../esphome/components/pulse_meter/pulse_meter_sensor.cpp -o test_pulse_meter

clean:
	rm test_pulse_meter