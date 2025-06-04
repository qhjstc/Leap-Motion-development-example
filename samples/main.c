/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <time.h>
#include "LeapC.h"
#include "ExampleConnection.h"
#include "cJSON.h"

#define FILE_PATH           "D:/cityu/imu/data/leapmotion/data.json"
#define WHOLE_SAMPLE_TIME   (6 * 60 * 1000) // 6 minutes in milliseconds

FILE* leapJsonFile;

// leap object
LEAP_CONNECTION* connHandle;
LEAP_CLOCK_REBASER clockSynchronizer;

// Add quaternion to json object
void jsonAddLeapQuaternion(cJSON* object, LEAP_BONE* bone)
{
    cJSON_AddNumberToObject(object, "w", bone->rotation.w);
    cJSON_AddNumberToObject(object, "x", bone->rotation.x);
    cJSON_AddNumberToObject(object, "y", bone->rotation.y);
    cJSON_AddNumberToObject(object, "z", bone->rotation.z);
}

char digitsName[5][10] = {"thumb", "index", "middle", "ring", "pinky"};
char digitsBonesName[4][15] = {"distal", "intermediate", "proximal", "metacarpal"};
void jsonAddLeapDigits(cJSON* object, LEAP_HAND* hand)
{
    cJSON* js_digit = NULL;
    // Store the rotation of each finger
    for(int i = 0; i < 5; i++){
        js_digit = cJSON_CreateObject();

        // Store the quaternion of each joint
        for(int j = 0; j < 4; j++) {
            if(i == 0 && j == 3)
                break;
            cJSON* json_bone = cJSON_CreateObject();
            jsonAddLeapQuaternion(json_bone, &hand->digits[i].bones[j]);
            cJSON_AddItemToObject(js_digit, digitsBonesName[j], json_bone);
        }
        cJSON_AddItemToObject(object, digitsName[i], js_digit);
    }
}


char* leapResultStoreJson(LEAP_TRACKING_EVENT* frame, long long timestamp)
{
    // Json object
    cJSON* cjson_object = NULL;
    if(frame->nHands == 0)
        return 0;
    cjson_object = cJSON_CreateObject();
    char *str;

    cJSON_AddLongLongToObject(cjson_object, "timestamp", timestamp);
    cJSON_AddLongLongToObject(cjson_object, "frame_id", frame->tracking_frame_id);
    cJSON_AddNumberToObject(cjson_object, "finger", 5);
    cJSON_AddNumberToObject(cjson_object, "hands", frame->nHands);

    for(uint32_t h = 0; h < frame->nHands; h++)
    {
        LEAP_HAND* hand = &frame->pHands[h];
        cJSON* cjson_hand = cJSON_CreateObject();

        jsonAddLeapDigits(cjson_hand, hand);
        cJSON_AddItemToObject(cjson_object, (hand->type == eLeapHandType_Left ? "left" : "right"), cjson_hand);
    }
    str = cJSON_Print(cjson_object);
    cJSON_free(cjson_object);

    return str;
}

int main(int argc, char** argv) {

    time_t current_time;
    struct tm* time_info;
    char time_string[9];

    // Get current time
    time(&current_time);
    time_info = localtime(&current_time);
    strftime(time_string, sizeof(time_string), "%H:%M:%S", time_info);

    printf("Leap motion data collection proecess starts! Current time is %s\n", time_string);
    connHandle = OpenConnection();

    while(!IsConnected){               // ??? why use this function to sleep, this function have some bug
        millisleep(250);
    }

    printf("Connected.\n");
    if(fopen_s(&leapJsonFile, FILE_PATH, "w") != 0)
    {
        printf("Can't open file normally!");
        exit(0);
    }
    fputs("[\n", leapJsonFile);
    //Create the clock synchronizer
    LeapCreateClockRebaser(&clockSynchronizer);
    clock_t cpuTime;
    int64_t targetFrameTime = 0;
    uint64_t targetFrameSize = 0;
    eLeapRS result;
    for(int i = 0;;){
        //Calculate the application time
        cpuTime = (clock_t).000001 * clock()/CLOCKS_PER_SEC;//microseconds
        //Synchronize the clocks
        LeapUpdateRebase(clockSynchronizer, cpuTime, LeapGetNow());

        //Simulate delay (i.e. processing load, v-sync, etc)
        millisleep(10);

        //Now get the updated application time
        cpuTime = (clock_t) .000001 * clock()/CLOCKS_PER_SEC;

        //Translate application time to Leap time
        LeapRebaseClock(clockSynchronizer, cpuTime, &targetFrameTime);

        //Get the buffer size needed to hold the tracking data
        result = LeapGetFrameSize(*connHandle, targetFrameTime, &targetFrameSize);
        if(result == eLeapRS_Success){
            //Allocate enough memory
            LEAP_TRACKING_EVENT* interpolatedFrame = malloc((size_t)targetFrameSize);
            //Get the frame
            result = LeapInterpolateFrame(*connHandle, targetFrameTime, interpolatedFrame, targetFrameSize);
            if(result == eLeapRS_Success){
                //Use the data...
//                printf("Frame %lli with %i hands with delay of %lli microseconds\n",
//                       (long long int)interpolatedFrame->tracking_frame_id,
//                       interpolatedFrame->nHands,
//                       (long long int)LeapGetNow() - interpolatedFrame->info.timestamp);
                if(interpolatedFrame->nHands == 0)
                    continue;
                // Just get an hour data
                SYSTEMTIME t1;
                GetSystemTime(&t1);
                long long int timestamp = time(NULL)%(60*60) * 1000 + (int)t1.wMilliseconds - (long long int)((long long int)LeapGetNow() - interpolatedFrame->info.timestamp)/1000;
                printf("Leap motion sample one frame! timestamp -- %lld\n", timestamp);

                char* str = leapResultStoreJson(interpolatedFrame, timestamp);
                fputs(str, leapJsonFile);
                free(str);
                //Free the allocated buffer when done.
                free(interpolatedFrame);
                if(timestamp > WHOLE_SAMPLE_TIME){
                    break;
                }
            }
            else {
//                printf("LeapInterpolateFrame() result was %s.\n", ResultString(result));
            }
        }
        else {
//            printf("LeapGetFrameSize() result was %s.\n", ResultString(result));
        }
    } //ctrl-c to exit
    fputs("\n]", leapJsonFile);
    fclose(leapJsonFile);
    printf("Leap motion process is end!\n");
    return 0;
}
//End-of-Sample
