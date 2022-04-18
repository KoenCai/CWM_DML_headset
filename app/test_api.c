#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

extern void CWM_OS_uSleep(uint32_t time);
extern uint64_t CWM_OS_GetTimeNs(void);
extern void* CWM_OS_malloc(int size);
extern void CWM_OS_free(void *ptr);
extern int CWM_OS_dbgOutput(const char * format);
extern int CWM_OS_dbgPrintf(const char * format,...);

int test_malloc_1() {
    char *pBuf1;
    int alloc_size;
    char buf1[0x100];

    CWM_OS_dbgPrintf("[test_malloc_1] start.\r\n");

    alloc_size = 15 * 1024;
    pBuf1 = (char*)CWM_OS_malloc(alloc_size);
    if (pBuf1 == NULL)
        goto __FAIL;

    memset(pBuf1, 0x7a, alloc_size);
    if (pBuf1[0] != 0x7a)
        goto __FAIL;

    if (pBuf1[alloc_size - 1] != 0x7a)
        goto __FAIL;

    strcpy(pBuf1, "success!!!!");

    sprintf(buf1, "[test_malloc_1] end. %s\r\n", pBuf1);
    CWM_OS_dbgPrintf(buf1);

    return 1;

__FAIL:
    CWM_OS_dbgPrintf("[test_malloc_1] end. failure!!!!\r\n");

    return 0;
}

int test_free_1() {
    char *pBuf1;
    char *pBuf2;
    int alloc_size;

    CWM_OS_dbgPrintf("[test_free_1] start.\r\n");

    alloc_size = 16;
    pBuf1 = (char*)CWM_OS_malloc(alloc_size);
    if(pBuf1 == NULL)
        goto __FAIL;

    //In none-OS or single thread system, if we use malloc() to acquire a memory block and free it immediately,
    //the next malloc() would get same memory block address
    //THIS IS A VERY ROUGHLY TEST
    CWM_OS_free(pBuf1);

    pBuf2 = (char*)CWM_OS_malloc(alloc_size);
    if(pBuf2 == NULL)
        goto __FAIL;

    if(pBuf1 == pBuf2)
        CWM_OS_dbgPrintf("[test_free_1] end. success!!!\r\n");
    else
        goto __FAIL;

    return 1;
__FAIL:
    CWM_OS_dbgPrintf("[test_free_1] end. failure!!!!\r\n");

    return 0;
}

void test_GetTimeNs_1() {
    uint64_t timeNow, timeLast, timeDiff;

    CWM_OS_dbgPrintf("[test_GetTimeNs_1] test 10 seconds start\r\n");

    timeLast = CWM_OS_GetTimeNs();
    while (1) {
        timeNow = CWM_OS_GetTimeNs();
        timeDiff = timeNow - timeLast;
        //CWM_OS_dbgPrintf("timeNow=%lld\ttimeLast=%lld\ttimeDiff=%lld\n", timeNow, timeLast, timeDiff);
        if (timeDiff >= (uint64_t)10 * 1000000000) {
            CWM_OS_dbgPrintf("[test_GetTimeNs_1] test 10 seconds end\r\n");
            break;
        }
    }
}

void test_GetTimeNs_2() {
    uint64_t timeNow, timeLast, timeDiff;
    uint64_t count;

    CWM_OS_dbgPrintf("[test_GetTimeNs_2] test 10 seconds start\r\n");

    timeLast = CWM_OS_GetTimeNs();
    count = 0;
    while (1) {
        timeNow = CWM_OS_GetTimeNs();
        timeDiff = timeNow - timeLast;
        //CWM_OS_dbgPrintf("timeNow=%lld\ttimeLast=%lld\ttimeDiff=%lld\n", timeNow, timeLast, timeDiff);
        if (timeDiff >= (uint64_t)1 * 1000000) {
            count ++;
            //CWM_OS_dbgPrintf("count=%lld\n", count);
            timeLast = timeNow;
            if (count >= (uint64_t)10 * 1000) {
                CWM_OS_dbgPrintf("[test_GetTimeNs_2] test 10 seconds end\r\n");
                break;
            }
        }
    }
}

void test_GetTimeNs_3() {
    uint64_t timeNow, timeLast, timeDiff;

    CWM_OS_dbgPrintf("[test_GetTimeNs_3] test 20 minutes start\r\n");

    timeLast = CWM_OS_GetTimeNs();
    while (1) {
        timeNow = CWM_OS_GetTimeNs();
        timeDiff = timeNow - timeLast;
        //CWM_OS_dbgPrintf("timeNow=%lld\ttimeLast=%lld\ttimeDiff=%lld\n", timeNow, timeLast, timeDiff);
        if (timeDiff >= (uint64_t)20 * 60 * 1000000000) {
            CWM_OS_dbgPrintf("[test_GetTimeNs_3] test 20 minutes end\r\n");
            break;
        }
    }
}

void test_uSleep_1() {

    CWM_OS_dbgPrintf("[test_uSleep_1] test 10 seconds start\r\n");

    CWM_OS_uSleep(10 * 1000000);

    CWM_OS_dbgPrintf("[test_uSleep_1] test 10 seconds end\r\n");
}

void test_uSleep_2() {
    int i, j;

    CWM_OS_dbgPrintf("[test_uSleep_2] test 10 seconds start\r\n");

    for (j = 0; j < 10; j ++) {
        for (i = 0; i < 1000; i++) {
            CWM_OS_uSleep(1000);
        }
    }

    CWM_OS_dbgPrintf("[test_uSleep_2] test 10 seconds end\r\n");
}

#ifndef CHECK_STACK_TIME_MS
#define CHECK_STACK_TIME_MS (uint64_t)(5 * 60 * 1000)
#endif

void test_stack() {
    // in FreeRTOS
    // set the config in FreeRTOSConfig.h file
    // #define configUSE_MALLOC_FAILED_HOOK 1
    // #define configCHECK_FOR_STACK_OVERFLOW 2
    // and implemnt vApplicationStackOverflowHook and vApplicationMallocFailedHook
    // the 2 api will be run while stack or memory allocate has problem

    volatile char buff[4096]; // stack size = 4096
    memset ((char*) buff, 0x96, sizeof(buff));
    uint64_t current_time = CWM_OS_GetTimeNs() / 1000000;
    uint64_t begin_time = CWM_OS_GetTimeNs() / 1000000;

    CWM_OS_dbgPrintf("[test_stack] test for %lld ms start\r\n", CHECK_STACK_TIME_MS);

    while (1) {
        for (int i=0; i< sizeof(buff); i++) {
            if (buff[i] != 0x96) {
                CWM_OS_dbgPrintf("[test_stack] somebody touch my stack!!!\r\n");
                CWM_OS_dbgPrintf("[test_stack] test for %lld ms end\r\n", CHECK_STACK_TIME_MS);
                return;
            }
        }

        CWM_OS_uSleep(200000);
        current_time = CWM_OS_GetTimeNs() / 1000000;

        if (current_time > begin_time && (current_time - begin_time > CHECK_STACK_TIME_MS)) {
            break;
        }
        else if (begin_time > current_time && (0xffffffffffffffff - begin_time + current_time) > CHECK_STACK_TIME_MS) {
            break;
        }
    }

    CWM_OS_dbgPrintf("[test_stack] test for %lld ms end\r\n", CHECK_STACK_TIME_MS);
}

void test_api() {

    test_malloc_1();
    test_free_1();
    test_GetTimeNs_1(); // 10 seconds
    test_GetTimeNs_2(); // 10 seconds
    test_GetTimeNs_3(); //20 minutes
    test_uSleep_1(); // 10 seconds
    test_uSleep_2(); // 10 seconds
}
