ALL_Q := 1 2 3 4 5
include ../common/rules.mk

.PHONY: run clean
run:
	@$(CXX) $(CXXFLAGS) $(QFLAGS) test.cpp -o quiz && ./quiz

clean:
	@rm -f quiz
