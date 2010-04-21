
#include <stdlib.h>
#include <string.h>

#include "mos.h"
#include "msched.h"
#include "printf.h"
#include "pub_sub.h"

#define BUF_SIZE 128
#define TEST_KEY 1

void test_pub_sub() {
	uint8_t buf[BUF_SIZE];
	
	pubSubInit(buf, BUF_SIZE);
	char *testStr = "testVal";
	pubSubPublish(TEST_KEY, testStr, sizeof(testStr));
	
	memset(testStr, 0, sizeof(testStr));
	pubSubRead(TEST_KEY, testStr);
	printf("testStr = %s \n", testStr);
}

void start(void)
{
   mos_thread_new(test_pub_sub, 192, PRIORITY_NORMAL);
}
