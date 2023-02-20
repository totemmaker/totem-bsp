/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef COMPONENTS_TOTEM_TEST
#define COMPONENTS_TOTEM_TEST

/**
 * Unit test library with additional features:
 * - Multiple source files support
 * - Test function execution time in microseconds
 * - Test ESP32 error, value range, list of values
 * - Test string with formatted match
 * - Fail test with formatted message
 * - Add formatted message for failed test
 * - Delay microseconds function
*/

#define TEST_RUN() TotemTest_run();

#define TEST_CASE(name, desc) static void TOTEM_TEST_FUNC(_test_case)();\
void __attribute__((constructor)) TOTEM_TEST_FUNC(_test_reg)(){\
static TotemTest_case_t test = {name " " desc, __FILE__, TOTEM_TEST_FUNC(_test_case), 0};\
TotemTest_register(&test);\
}\
static void TOTEM_TEST_FUNC(_test_case)()

#define TEST_ASSERT(condition) TotemTest_assert(condition, "TEST_ASSERT(" #condition ")", __LINE__)
#define TEST_ERROR(expected, actual) TotemTest_error(expected, actual, "TEST_ERROR(" #expected ", " #actual ")", __LINE__)
#define TEST_MAX(maximum, actual) TotemTest_max(maximum, actual, "TEST_MAX(" #maximum ", " #actual ")", __LINE__)
#define TEST_MIN(expected, actual) TotemTest_min(expected, actual, "TEST_MIN(" #expected ", " #actual ")", __LINE__)
#define TEST_FLOAT(expected, drift, actual) TotemTest_float(expected, drift, actual, "TEST_FLOAT(" #expected ", " #drift ", " #actual ")", __LINE__)
#define TEST_EQUAL(expected, actual) TotemTest_equal(expected, actual, "TEST_EQUAL(" #expected ", " #actual ")", __LINE__)
#define TEST_EQUALF(expected, actual) TotemTest_equal_float(expected, actual, "TEST_EQUALF(" #expected ", " #actual ")", __LINE__)
#define TEST_NOT_EQUAL(not_expected, actual) TotemTest_not_equal(not_expected, actual, "TEST_NOT_EQUAL(" #not_expected ", " #actual ")", __LINE__)
#define TEST_LIMIT(from, to, actual) TotemTest_range(from, to, actual, "TEST_LIMIT(" #from ", " #to ", " #actual ")", __LINE__)
#define TEST_STRING(str1, ...) TotemTest_string("TEST_STRING(" #str1 ", " #__VA_ARGS__")", __LINE__, str1, __VA_ARGS__)
#define TEST_ARRAY(array, size, actual) TotemTest_array(array, size, actual, "TEST_ARRAY(" #array ", " #size ", " #actual ")", __LINE__)
#define TEST_VALUES(...){\
int32_t valueList[] = {__VA_ARGS__};\
TotemTest_array(valueList, (sizeof(valueList)/4)-1, valueList[(sizeof(valueList)/4)-1], "TEST_VALUES(" #__VA_ARGS__ ")", __LINE__);}

#define TEST_PRINT(...) TotemTest_print(__VA_ARGS__)
#define TEST_MESSAGE(...) TotemTest_set_message(__VA_ARGS__)
#define TEST_FAIL(...) TotemTest_fail("TEST_FAIL(" #__VA_ARGS__ ")", __LINE__, __VA_ARGS__)
#define TEST_DELAY(ms) TotemTest_delay(ms)
#define TEST_TIME_START() TotemTest_time_start()
#define TEST_TIME_END() TotemTest_time_end()
#define TEST_TIME(time_micros, expression) {\
TotemTest_time_start();\
expression;\
TotemTest_time_end();\
TotemTest_time(time_micros, TotemTest_time_get(), "TEST_TIME(" #time_micros ", " #expression ")", __LINE__);\
}\

#define TOTEM_TEST_CONCAT(a,b,c) a##_##b##_##c##_
#define TOTEM_TEST_EXPAND(a,b,c) TOTEM_TEST_CONCAT(a,b,c)
#define TOTEM_TEST_FUNC(prefix) TOTEM_TEST_EXPAND(prefix, TOTEM_INCLUDE_LEVEL, __LINE__)

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *name;
    const char *file;
    void (*func)();
    void *next;
} TotemTest_case_t;

void TotemTest_register(TotemTest_case_t *test);
void TotemTest_assert(int32_t result,                          const char *expression, uint32_t line);
void TotemTest_error(esp_err_t expected, esp_err_t actual,     const char *expression, uint32_t line);
void TotemTest_max(int32_t maximum, int32_t actual,            const char *expression, uint32_t line);
void TotemTest_min(int32_t minimum, int32_t actual,            const char *expression, uint32_t line);
void TotemTest_float(float expected, float drift, float actual,const char *expression, uint32_t line);
void TotemTest_equal(int32_t expected, int32_t actual,         const char *expression, uint32_t line);
void TotemTest_equal_float(float expected, float actual,       const char *expression, uint32_t line);
void TotemTest_not_equal(int32_t not_expected, int32_t actual, const char *expression, uint32_t line);
void TotemTest_range(int32_t from, int32_t to, int32_t actual, const char *expression, uint32_t line);
void TotemTest_string(const char *expression, uint32_t line, const char *str1, const char *str2, ...);
void TotemTest_array(int32_t *array, uint32_t size, int32_t actual, const char *expression, uint32_t line);
void TotemTest_time(uint32_t expected, uint32_t actual,        const char *expression, uint32_t line);
void TotemTest_fail(const char *expression, uint32_t line, const char *format, ...);
void TotemTest_print(const char *format, ...);

void TotemTest_time_start();
uint32_t TotemTest_time_end();
uint32_t TotemTest_time_get();

void TotemTest_set_message(const char *message, ...);
void TotemTest_delay(uint32_t ms);
void TotemTest_terminate();
void TotemTest_run();

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_TOTEM_TEST */

#if TOTEM_INCLUDE_LEVEL == 8
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 9
#elif TOTEM_INCLUDE_LEVEL == 7
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 8
#elif TOTEM_INCLUDE_LEVEL == 6
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 7
#elif TOTEM_INCLUDE_LEVEL == 5
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 6
#elif TOTEM_INCLUDE_LEVEL == 4
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 5
#elif TOTEM_INCLUDE_LEVEL == 3
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 4
#elif TOTEM_INCLUDE_LEVEL == 2
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 3
#elif TOTEM_INCLUDE_LEVEL == 1
#undef TOTEM_INCLUDE_LEVEL
#define TOTEM_INCLUDE_LEVEL 2
#else
#define TOTEM_INCLUDE_LEVEL 1
#endif