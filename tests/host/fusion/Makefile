CC ?= cc
CFLAGS ?= -O1 -g -std=c11 -Wall -Wextra -Werror
CFLAGS += -DHOST_FUSION_TEST -fsanitize=undefined -fno-sanitize-recover=undefined -I. -I../../../src -include zephyr/kernel.h
LDLIBS += -lm -fsanitize=undefined

.PHONY: check clean

TESTS := test_eqf_mag_hold test_eqf_rest_heading

check: $(TESTS)
	./test_eqf_mag_hold
	./test_eqf_rest_heading

$(TESTS): %: %.c ../../../src/sensor/fusion/eqf/eqf.c
	$(CC) $(CFLAGS) -o $@ $< ../../../src/sensor/fusion/eqf/eqf.c $(LDLIBS)

clean:
	$(RM) $(TESTS)
