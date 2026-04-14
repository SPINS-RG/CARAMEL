install:
	apt-get install bison pkg-config gawk clang flex gcc-msp430 iverilog tcl-dev

ultrasonic_sensor:
	rm -f ../logs/*.cflog
	sh build/run "ultrasonic_sensor" true
	
test:
	rm -f ../logs/*.cflog
	sh build/run "test" true

temperature_sensor:
	rm -f ../logs/*.cflog
	sh build/run "temperature_sensor" true

syringe_pump:
	rm -f ../logs/*.cflog
	sh build/run "syringe_pump" true
	
rover:
	sh build/run "rover" true

clean :
	rm -rf simv *.mem pmem.inc stimulus.v tmp-build
