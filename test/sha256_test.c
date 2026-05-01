#include <stdio.h>
#include <stdlib.h>
#include <openssl/sha.h>
#include <string.h>

#include "../src/crypto.h"

#define BOLDRED     "\033[1m\033[31m"
#define BOLDGREEN   "\033[1m\033[32m"
#define ENDCOLOUR "\033[0m"

// Print hash
void print_sha256(unsigned char* hash) {
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
        printf("%02x", hash[i]);
    }

    printf("\n");
}

// Test equality of two hash's
bool test_sha256(unsigned char* hash1, unsigned char* hash2) {
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
		if(hash1[i] != hash2[i]) {
			return false;
		}
    }
	return true;
}

int main() {
    const char *msg = "Here is some random crazy number 437985743985743";

    unsigned char hash[SHA256_DIGEST_LENGTH];
    unsigned char our_hash[SHA256_DIGEST_LENGTH];

	// Call openSSL sha256
    SHA256((const unsigned char *)msg, strlen(msg), hash);
	print_sha256(hash);

	// Our sha256
	sha256((const unsigned char *)msg, strlen(msg), our_hash);
	print_sha256(our_hash);

	// Test
	if(test_sha256(hash, our_hash)) {
		printf(BOLDGREEN "Test Passed\n" ENDCOLOUR);
	} else {
		printf(BOLDRED "Test Failed\n" ENDCOLOUR);
	}

    return 0;
}
