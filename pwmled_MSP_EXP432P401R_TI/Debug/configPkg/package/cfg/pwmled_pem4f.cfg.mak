# invoke SourceDir generated makefile for pwmled.pem4f
pwmled.pem4f: .libraries,pwmled.pem4f
.libraries,pwmled.pem4f: package/cfg/pwmled_pem4f.xdl
	$(MAKE) -f /home/abenbihi/courses/6561/hmw3/ECE6561-Project2/ws/pwmled_MSP_EXP432P401R_TI/src/makefile.libs

clean::
	$(MAKE) -f /home/abenbihi/courses/6561/hmw3/ECE6561-Project2/ws/pwmled_MSP_EXP432P401R_TI/src/makefile.libs clean

