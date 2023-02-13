/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <stdio.h>
#include <inttypes.h>
#include <stdarg.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_timer.h"
#include "totem_test.h"

static TotemTest_case_t *first_test_ptr;
static char messageBuffer[200];
static uint32_t checksCount;
static int64_t startTime, endTime;
static const char *currFile;

#define PASS() messageBuffer[0]=0;checksCount++;return
#define FAIL() TotemTest_terminate()

// Print file name with line number
static void print_file(uint32_t line) {
    printf("File : %s:%" PRId32 "\n", currFile, line);
}
// Print test expression
static void print_expression(const char *expression) {
    printf("Test : %s\n", expression);
}

void TotemTest_assert(int32_t result, const char *expression, uint32_t line) {
    if (result) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Condition failed (false).\n");
    FAIL();
}
void TotemTest_error(esp_err_t expected, esp_err_t actual, const char *expression, uint32_t line) {
    if (expected == actual) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected %s (0x%" PRIX32 ") but %s (0x%" PRIX32 ") received.\n", esp_err_to_name(expected), (uint32_t)expected, esp_err_to_name(actual), (uint32_t)actual);
    FAIL();
}
void TotemTest_max(int32_t maximum, int32_t actual, const char *expression, uint32_t line) {
    if (maximum >= actual) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected %" PRId32 " (0x%" PRIX32 ") to be maximum but %" PRId32 " (0x%" PRIX32 ") received.\n", maximum, (uint32_t)maximum, actual, (uint32_t)actual);
    FAIL();
}
void TotemTest_min(int32_t minimum, int32_t actual, const char *expression, uint32_t line) {
    if (minimum <= actual) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected %" PRId32 " (0x%" PRIX32 ") to be minimum but %" PRId32 " (0x%" PRIX32 ") received.\n", minimum,(uint32_t)minimum, actual, (uint32_t)actual);
    FAIL();
}
void TotemTest_equal(int32_t expected, int32_t actual, const char *expression, uint32_t line) {
    if (expected == actual) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected %" PRId32 " (0x%" PRIX32 ") but %" PRId32 " (0x%" PRIX32 ") received.\n", expected, (uint32_t)expected, actual, (uint32_t)actual);
    FAIL();
}
void TotemTest_not_equal(int32_t not_expected, int32_t actual, const char *expression, uint32_t line) {
    if (not_expected != actual) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Not expected %" PRId32 " (0x%" PRIX32 ") to match %" PRId32 " (0x%" PRIX32 ") received.\n", not_expected, (uint32_t)not_expected, actual, (uint32_t)actual);
    FAIL();
}
void TotemTest_range(int32_t from, int32_t to, int32_t actual, const char *expression, uint32_t line) {
    if (from <= to) { if (actual >= from && actual <= to) { PASS(); } }
    else if (actual >= to && actual <= from) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected limit in range [%" PRId32 ":%" PRId32 "] but %" PRId32 " (0x%" PRIX32 ") received.\n", from, to, actual, (uint32_t)actual);
    FAIL();
}
void TotemTest_string(const char *expression, uint32_t line, const char *str1, const char *str2, ...) {
    char buffer[200];
    va_list args;
    va_start(args, str2);
    vsnprintf(buffer, sizeof(buffer), str2, args);
    va_end(args);
    if (strncmp(str1, buffer, sizeof(buffer)) == 0) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected \"%s\" to be equal \"%s\"\n", str1, buffer);
    FAIL();
}
void TotemTest_array(int32_t *array, uint32_t size, int32_t actual, const char *expression, uint32_t line) {
    int i;
    for (i=0; i<size; i++) {
        if (array[i] == actual) { PASS(); }
    }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected value in list [ ");
    for (i=0; i<size; i++) {
        printf("%" PRId32 " ", array[i]);
    }
    printf("] but %" PRId32 " (0x%" PRIX32 ") received.\n", actual, (uint32_t)actual);
    FAIL();
}
void TotemTest_time(uint32_t expected, uint32_t actual, const char *expression, uint32_t line) {
    if (expected >= actual) { PASS(); }
    print_file(line);
    print_expression(expression);
    printf("Error: Expected execute in %" PRIu32 " but took %" PRIu32 " microseconds.\n", expected, actual);
    FAIL();
}
void TotemTest_fail(const char *expression, uint32_t line, const char *format, ...) {
    va_list args;
    print_file(line);
    print_expression(expression);
    printf("Error: ");
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
    FAIL();
}
void TotemTest_print(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}
void IRAM_ATTR TotemTest_time_start() {
    startTime = esp_timer_get_time();
}
uint32_t IRAM_ATTR TotemTest_time_end() {
    endTime = esp_timer_get_time();
    return endTime-startTime;
}
uint32_t TotemTest_time_get() {
    return endTime-startTime;
}
void TotemTest_set_message(const char *message, ...) {
    va_list args;
    va_start(args, message);
    vsnprintf(messageBuffer, sizeof(messageBuffer), message, args);
    va_end(args);
}
void TotemTest_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
void TotemTest_terminate() {
    if (messageBuffer[0]) { printf("Message: %s.\n", messageBuffer); }
    vTaskSuspend(xTaskGetCurrentTaskHandle());
}
void TotemTest_run() {
    uint32_t caseId = 0;
    for (TotemTest_case_t *ptr = first_test_ptr; ptr != NULL; ptr=(TotemTest_case_t*)ptr->next) {
        caseId++;
        currFile = ptr->file;
        printf("Running test #%" PRIu32 " \"%s\".\n", caseId, ptr->name);
        ptr->func();
        printf("PASSED\n");
    }
    printf("------------------------------------------\n");
    printf("All tests SUCCESS. Cases: %" PRIu32 ", checks: %" PRIu32 ".\n", caseId, checksCount);
    printf("------------------------------------------\n");
}
void TotemTest_register(TotemTest_case_t *test) {
    test->next = first_test_ptr;
    first_test_ptr = test;
}
