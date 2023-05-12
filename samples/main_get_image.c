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
#include "LeapC.h"
#include "ExampleConnection.h"

#include <Windows.h>
#include <time.h>
#include "cJSON.h"

LEAP_CONNECTION *connection;

char COLLECT_DATA_MODE[10] = "default";
long long int WHOLE_SAMPLE_TIME = 6 * 60 * 1000;
long long START_TIME = 0;
char leapmotion_store_file_name[256] = "D:/cityu/finger-tracking/software/leapmotion/leap-example/leap-data/data.json";

FILE* leapJsonFile;
typedef struct Quaternion {
    double w, x, y, z;
} Quaternion;

typedef struct Matrix {
    double m[3][3];
} Matrix;

void quaternionToRotationMatrix(Quaternion q, Matrix *m) {
    double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    double s = (n > 0.0) ? (2.0 / n) : 0.0;
    double wx = s*q.w*q.x, wy = s*q.w*q.y, wz = s*q.w*q.z;
    double xx = s*q.x*q.x, xy = s*q.x*q.y, xz = s*q.x*q.z;
    double yy = s*q.y*q.y, yz = s*q.y*q.z, zz = s*q.z*q.z;

    m->m[0][0] = 1.0 - (yy + zz);
    m->m[0][1] = xy - wz;
    m->m[0][2] = xz + wy;

    m->m[1][0] = xy + wz;
    m->m[1][1] = 1.0 - (xx + zz);
    m->m[1][2] = yz - wx;

    m->m[2][0] = xz - wy;
    m->m[2][1] = yz + wx;
    m->m[2][2] = 1.0 - (xx + yy);
}

void readParamFile(){
    FILE* fp;

    // 打开文件
    fopen_s(&fp, "D:/cityu/finger-tracking/software/file-control/param.txt", "r");
    if (fp == NULL) {
        printf("Leap process: Failed to open file\n");
        return;
    }

    char line[256];
    char name[100], value[256];

    while (fgets(line, 256, fp) != NULL) {
        // 去掉行尾的换行符
        line[strcspn(line, "\n")] = 0;
        // 按等号分割字符串
        char* token;
        char* next_token;
        token = strtok_s(line, " = ", &next_token);
        strcpy_s(name, 100, token);
        token = strtok_s(NULL, " = ", &next_token);
        strcpy_s(value, 256, token);
//        // 去掉名称和值周围的引号
//        memmove(name, name + 1, strlen(name) - 2);
//        memmove(value, value + 1, strlen(value) - 2);
        // 根据名称赋值给对应的变量
        if (strcmp(name, "WHOLE_SAMPLE_TIME") == 0) {
            WHOLE_SAMPLE_TIME = atoi(value);
            printf("Leap process: WHOLE_SAMPLE_TIME = %d\n", WHOLE_SAMPLE_TIME);
        }
        else if (strcmp(name, "leapmotion_store_file_name") == 0) {
            int valid_length = strlen(value) - 2;
            memset(leapmotion_store_file_name, 0, sizeof(leapmotion_store_file_name));
            for(int i = 0; i < valid_length; i++){
                leapmotion_store_file_name[i] = value[i+1];
            }
            // ...
        }
        else if (strcmp(name, "COLLECT_DATA_MODE") == 0) {
            if (strcmp(value, "\"default\"") == 0) {
                printf("Leap process: Mode = %s\n", value);
                break;
            }
        }
    }

    fclose(fp);
}

// Add quaternion to json object
void jsonAddLeapQuaternion(cJSON* object, LEAP_QUATERNION* rotation)
{
    cJSON_AddNumberToObject(object, "w", rotation->w);
    cJSON_AddNumberToObject(object, "x", rotation->x);
    cJSON_AddNumberToObject(object, "y", rotation->y);
    cJSON_AddNumberToObject(object, "z", rotation->z);
}


char digitsName[5][10] = {"thumb", "index", "middle", "ring", "pinky"};
char digitsBonesName[4][15] = {"metacarpal", "proximal", "intermediate", "distal"};
void jsonAddLeapDigits(cJSON* object, LEAP_HAND* hand)
{
    cJSON* js_digit = NULL;

    // Store the rotation of each finger
    for(int i = 0; i < 5; i++){
        js_digit = cJSON_CreateObject();

        // Store the quaternion of each joint
        for(int j = 0; j < 4; j++) {
            if(i == 0 && j == 0)
                break;
            cJSON* json_bone = cJSON_CreateObject();
            jsonAddLeapQuaternion(json_bone, &hand->digits[i].bones[j].rotation);
            cJSON_AddItemToObject(js_digit, digitsBonesName[j], json_bone);
        }
        cJSON_AddItemToObject(object, digitsName[i], js_digit);
    }
}


char* leapResultStoreJson(const LEAP_TRACKING_EVENT* frame, long long timestamp)
{
    // Json object
    cJSON* cjson_object = NULL;
    if(frame->nHands == 0)
        return 0;
    cjson_object = cJSON_CreateObject();
    char *str;

    LEAP_DEVICE_REF *device_ref_array = NULL;
    uint32_t* ref_count = 0;
    //Open device using LEAP_DEVICE_REF from event struct.
    LeapGetDeviceList(connection[0], device_ref_array, ref_count);
    LEAP_DEVICE deviceHandle;
    eLeapRS result = LeapOpenDevice(device_ref_array[0], &deviceHandle);
    if(result != eLeapRS_Success){
        printf("Could not open device %s.\n", ResultString(result));
        return 0;
    }


    cJSON_AddLongLongToObject(cjson_object, "timestamp", timestamp);
    cJSON_AddLongLongToObject(cjson_object, "frame_id", frame->tracking_frame_id);
    cJSON_AddNumberToObject(cjson_object, "finger", 5);
    cJSON_AddNumberToObject(cjson_object, "hands", frame->nHands);

    for(uint32_t h = 0; h < frame->nHands; h++)
    {
        LEAP_HAND* hand = &frame->pHands[h];
        cJSON* cjson_hand = cJSON_CreateObject();
        // Get the Leap transform
        float leap_transform[16];

        leap_transform[0] = hand->palm.position.x;

        LeapGetDeviceTransform(deviceHandle, leap_transform);

        cJSON* cjson_wrist = cJSON_CreateObject();
        jsonAddLeapQuaternion(cjson_wrist, &hand->palm.orientation);
        cJSON_AddItemToObject(cjson_hand, "wrist", cjson_wrist);
        jsonAddLeapDigits(cjson_hand, hand);
        cJSON_AddItemToObject(cjson_object, (hand->type == eLeapHandType_Left ? "left" : "right"), cjson_hand);
    }
    str = cJSON_Print(cjson_object);
    cJSON_free(cjson_object);

    LeapCloseDevice(deviceHandle);

    return str;
}

/** Callback for when the connection opens. */
static void OnConnect(void){
    printf("Connected.\n");
    SYSTEMTIME t1;
    GetSystemTime(&t1);
    struct tm local_time;
    time_t current_time;
    // 获取当前时间
    time(&current_time);
    // 将当前时间转换为本地时间
    localtime_s(&local_time, &current_time);
    START_TIME = (local_time.tm_hour*60*60+local_time.tm_min*60+local_time.tm_sec) * 1000 + (int)t1.wMilliseconds;
}

/** Callback for when a device is found. */
static void OnDevice(const LEAP_DEVICE_INFO *props){
    printf("Found device %s.\n", props->serial);
}

long long flag_time = 0;
/** Callback for when a frame of tracking data is available. */

static void OnFrame(const LEAP_TRACKING_EVENT *frame){
    static int storeFrameNumber = 0;
    SYSTEMTIME t1;
    GetSystemTime(&t1);
    struct tm local_time;
    time_t current_time;
    // 获取当前时间
    time(&current_time);
    // 将当前时间转换为本地时间
    localtime_s(&local_time, &current_time);
    long long int timestamp = (local_time.tm_hour*60*60+local_time.tm_min*60+local_time.tm_sec) * 1000 + (int)t1.wMilliseconds - (long long int)((long long int)LeapGetNow() - frame->info.timestamp)/1000;
    if(timestamp > (START_TIME + WHOLE_SAMPLE_TIME + 10000)) {
        fputs("\n]", leapJsonFile);
        fclose(leapJsonFile);
//        CloseConnection();
//        DestroyConnection();
        printf("Leap process: Leap motion process is end!\n");
        printf("Press Enter to exit...\n");

        // Waiting enter to exit
        while(getchar() != '\n');

        exit(0);
    }

    if(frame->nHands == 0) {
        if(timestamp > (flag_time+100)) {
            flag_time = timestamp;
            printf("Leap process: No hands! timestamp--%lld\n", timestamp);
        }
        return;
    }
    if(storeFrameNumber > 0){
        fputs(",\n", leapJsonFile);
    }
    storeFrameNumber++;
    if(timestamp > (flag_time+100)) {
        flag_time = timestamp;
        printf("Leap process: Leap motion sample one frame! timestamp--%lld\n", timestamp);
    }
    char* str = leapResultStoreJson(frame, timestamp);

    fputs(str, leapJsonFile);

    free(str);

}

/** Callback for when an image is available. */
static void OnImage(const LEAP_IMAGE_EVENT *imageEvent){
//    printf("Received image set for frame %lli with size %lli.\n",
//           (long long int)imageEvent->info.frame_id,
//           (long long int)imageEvent->image[0].properties.width*
//           (long long int)imageEvent->image[0].properties.height*2);
}

int main(int argc, char** argv) {
    char buf[1024];
    setvbuf(stdout, buf, _IONBF, sizeof(buf));
    readParamFile();

    // Calculate current time
    time_t current_time;
    struct tm* time_info;
    char time_string[9];
    // 获取当前时间
    time(&current_time);
    time_info = localtime(&current_time);
    // 格式化时间字符串
    strftime(time_string, sizeof(time_string), "%H:%M:%S", time_info);

    printf("Leap process: Leap motion data collection proecess starts! Current time is %s\n", time_string);

    printf("%s\n", leapmotion_store_file_name);
    // Creat store json file
    if(fopen_s(&leapJsonFile, leapmotion_store_file_name, "w") != 0){
        printf("Leap process: Can't open file normally!\n");
        exit(0);
    }
    fputs("[\n", leapJsonFile);

    //Set callback function pointers
    ConnectionCallbacks.on_connection          = &OnConnect;
    ConnectionCallbacks.on_device_found        = &OnDevice;
    ConnectionCallbacks.on_frame               = &OnFrame;
    ConnectionCallbacks.on_image               = &OnImage;

    connection = OpenConnection();
    LeapSetPolicyFlags(*connection, eLeapPolicyFlag_Images, 0);

    while(1){

    }
    return 0;
}
//End-of-Sample
