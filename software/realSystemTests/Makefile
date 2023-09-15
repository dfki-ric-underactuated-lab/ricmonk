TEST = tvlqr
FILE = BF_tvlqr.csv
MOTOR_ID = [8,9]
BUS_NUMBER = 2
FILE_RECOVERY = ZF_tvlqr.csv
INIT_CONF=1


.PHONY: test
test:
	sudo -E ./test_multiple_brachiations.py $(TEST) $(FILE) $(BUS_NUMBER) $(MOTOR_ID) $(FILE_RECOVERY) $(INIT_CONF)
