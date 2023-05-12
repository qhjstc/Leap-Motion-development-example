/* Copyright (C) 2021 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */

#include "LeapC.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// If an error occurs, record the error message
static void assert_impl(bool success, const char* instruction_line)
{
  if (!success)
  {
    fprintf(stderr, "ERROR: %s\n", instruction_line);
    abort();
  }
}

#define str_i(s) #s
#define str(s) str_i(s)

#define assert_i(test, msg) assert_impl(test, __FILE__ ":" str(__LINE__) " -- " msg)
#define assert(test) assert_i((test), #test)

int main(int argc, const char** argv)
{
  LEAP_CONNECTION connectionHandle;   // Create a leap connection handle

  assert(LeapCreateConnection(NULL, &connectionHandle) == eLeapRS_Success); // Create leap connection
  assert(LeapOpenConnection(connectionHandle) == eLeapRS_Success);  // Open leap connection

  const uint32_t timeout = 1000U;   // Set up timeout time

  uint32_t computed_array_size = 0U;    // The number of connected leap

  for (uint32_t retry_limit = 7U; retry_limit > 0; --retry_limit)
  {
    LEAP_CONNECTION_MESSAGE msg;    //  Create a connection message

    LeapPollConnection(connectionHandle, timeout, &msg);

    eLeapRS return_code = LeapGetDeviceList(connectionHandle, NULL, &computed_array_size);
    if (return_code == eLeapRS_NotConnected)
    {
      continue;
    }
    assert(return_code == eLeapRS_Success);
    printf("Number of devices available: %u\n", computed_array_size);

    if (computed_array_size > 0U)   // if the connection is successful, push out the connection mode
    {
      break;
    }
  }

  if (computed_array_size > 0U)
  {
    LEAP_DEVICE_REF* leap_device_list =
      (LEAP_DEVICE_REF*)malloc(sizeof(LEAP_DEVICE_REF) * computed_array_size);
    assert(
      LeapGetDeviceList(connectionHandle, leap_device_list, &computed_array_size) == eLeapRS_Success);

    /* Make use of leap_device_list here */

    free(leap_device_list);
  }

  LeapCloseConnection(connectionHandle);
  LeapDestroyConnection(connectionHandle);

  return eLeapRS_Success;
}
