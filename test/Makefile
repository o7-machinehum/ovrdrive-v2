CC = gcc
CFLAGS = -Wall -Wextra -O2
LDLIBS = -lcrypto

sha256_test: sha256_test.o crypto.o
	$(CC) $^ $(LDLIBS) -o $@

sha256_test.o: sha256_test.c
	$(CC) $(CFLAGS) -c $< -o $@

crypto.o: ../src/crypto.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f sha256_test *.o

test: sha256_test
	./sha256_test
