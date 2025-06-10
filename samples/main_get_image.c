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
#include "math.h"
#include "cJSON.h"


#define CONTROL_MOUSE   TRUE

LEAP_CONNECTION *connection = NULL;

char COLLECT_DATA_MODE[10] = "default";
long long int WHOLE_SAMPLE_TIME = 6 * 60 * 1000;
long long START_TIME = 0;
char store_file_name[256] = "D:/cityu/imu/data/leapmotion/data.json";

FILE* leapJsonFile;
typedef struct Quaternion {
    float w, x, y, z;
} Quaternion;

typedef struct Location {
    float x, y, z;
} Location;

// 计算两个四元数相乘
Quaternion multiplyQuaternion(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    return result;
}


void quaternionTransformCoordinate(Quaternion rotationQ, Quaternion *localQ) {
    Quaternion result;
    Quaternion conjugateQ = {rotationQ.w, -rotationQ.x, -rotationQ.y, -rotationQ.z};
    result = multiplyQuaternion(conjugateQ, *localQ);
    localQ->w = result.w;
    localQ->x = result.x;
    localQ->y = result.y;
    localQ->z = result.z;
}

/** ======================================================================================================================== **/

void readParamFile(const struct tm* time_info){
    FILE* fp;

    // 打开文件
    fopen_s(&fp, "../param.txt", "r");
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
        // 根据名称赋值给对应的变量
        if (strcmp(name, "WHOLE_SAMPLE_TIME") == 0) {
            WHOLE_SAMPLE_TIME = atoi(value);
            printf("Leap process: WHOLE_SAMPLE_TIME = %d\n", WHOLE_SAMPLE_TIME);
        }
        else if (strcmp(name, "STORE_FILE_NAME") == 0) {
            int valid_length = strlen(value);
            memset(store_file_name, 0, sizeof(store_file_name));
            char file_time_string[20];
            strftime(file_time_string, sizeof(file_time_string), "_%m_%d_%H_%M_%S", time_info);
            for(int i = 0, j = 0; i < valid_length; i++){
                if (value[i] == '.') {
                    for (int k = 0; k < strlen(file_time_string); k++) {
                        store_file_name[j++] = file_time_string[k];
                    }
                }
                store_file_name[j++] = value[i];
            }

            printf("Leap process: STORE_FILE_NAME = %s\n", store_file_name);
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

    fp = fopen("../setting.txt", "w"); // 以追加模式打开
    if (fp == NULL) {
        printf("Leap process: Failed to open file for append\n");
        return;
    }
    fprintf(fp, "STORE_FILE_NAME_NEW = %s\n", store_file_name);

    fclose(fp); // 关闭写入流
}

// Add location to json object
void jsonAddLeapLocation(cJSON* object, Location* location)
{
    cJSON_AddNumberToObject(object, "x", location->x);
    cJSON_AddNumberToObject(object, "y", location->y);
    cJSON_AddNumberToObject(object, "z", location->z);
}

// Add quaternion to json object
void jsonAddLeapQuaternion(cJSON* object, Quaternion* rotation)
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
                continue;
            cJSON* json_bone = cJSON_CreateObject();
            if(j > 0) {
                Quaternion preQ;
                if(i == 0){     // Thumb use the palm coordinate system
                    LEAP_QUATERNION preBoneQ = hand->palm.orientation;
                    preQ.w = preBoneQ.w;
                    preQ.x = preBoneQ.x;
                    preQ.y = preBoneQ.y;
                    preQ.z = preBoneQ.z;
                }
                else {        // Other fingers
                    LEAP_QUATERNION preBoneQ = hand->digits[i].bones[j - 1].rotation;
                    preQ.w = preBoneQ.w;
                    preQ.x = preBoneQ.x;
                    preQ.y = preBoneQ.y;
                    preQ.z = preBoneQ.z;
                }
                LEAP_QUATERNION boneQ = hand->digits[i].bones[j].rotation;
                Quaternion localQ = {boneQ.w, boneQ.x, boneQ.y, boneQ.z};
                quaternionTransformCoordinate(preQ, &localQ);
                jsonAddLeapQuaternion(json_bone, &localQ);
                cJSON_AddItemToObject(js_digit, digitsBonesName[j], json_bone);
            }
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


    // Store frame information
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

        cJSON* cjson_position = cJSON_CreateObject();
        Location position = {hand->palm.position.x, hand->palm.position.y, hand->palm.position.z};
        jsonAddLeapLocation(cjson_position, &position);
        cJSON_AddItemToObject(cjson_hand, "position", cjson_position);
        cJSON* cjson_wrist = cJSON_CreateObject();
        Quaternion q = {hand->palm.orientation.w, hand->palm.orientation.x, hand->palm.orientation.y, hand->palm.orientation.z};
        jsonAddLeapQuaternion(cjson_wrist, &q);
        cJSON_AddItemToObject(cjson_hand, "wrist", cjson_wrist);
        jsonAddLeapDigits(cjson_hand, hand);
        cJSON_AddItemToObject(cjson_object, (hand->type == eLeapHandType_Left ? "left" : "right"), cjson_hand);

        if (CONTROL_MOUSE) {
            const int max_width = GetSystemMetrics(SM_CXSCREEN);
            const int max_height = GetSystemMetrics(SM_CYSCREEN);
            const int virtual_width = 200; // mm

            static int mouse_x = 0;
            static int mouse_y = 0;
            static int initialized = 0;
            static float last_position_x = 0;
            static float last_position_y = 0;

            float position_x = hand->palm.position.x;
            float position_y = hand->palm.position.y;

            if (!initialized) {
                mouse_x = max_width / 2;
                mouse_y = max_height / 2;
                last_position_x = position_x;
                last_position_y = position_y;
                initialized = 1;
            }

            float offset_pos_x = position_x - last_position_x;
            float offset_pos_y = position_y - last_position_y;

            int offset_x = (int)(offset_pos_x / (float)virtual_width * (float)max_width);
            int offset_y = (int)(offset_pos_y / (float)virtual_width * (float)max_height);

            static float total_offset_x = 0;
            static float total_offset_y = 0;

            total_offset_x += offset_pos_x;
            total_offset_y += offset_pos_y;

            mouse_x = mouse_x + offset_x;
            mouse_y = mouse_y - offset_y;

            if (mouse_x < 0) mouse_x = 0;
            if (mouse_x >= max_width) mouse_x = max_width - 1;
            if (mouse_y < 0) mouse_y = 0;
            if (mouse_y >= max_height) mouse_y = max_height - 1;

            last_position_x = position_x;
            last_position_y = position_y;

            SetCursorPos(mouse_x, mouse_y);
        }
    }
    str = cJSON_Print(cjson_object);
    cJSON_free(cjson_object);

    return str;
}

/** ======================================================================================================================== **/

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


/** ======================================================================================================================== **/
int main(int argc, char** argv) {
    char buf[1024];
    setvbuf(stdout, buf, _IONBF, sizeof(buf));

    // Get current time
    time_t current_time;
    struct tm* time_info;
    char time_string[9];
    time(&current_time);
    time_info = localtime(&current_time);
    strftime(time_string, sizeof(time_string), "%H:%M:%S", time_info);

    printf("Leap process: Leap motion data collection proecess starts! Current time is %s\n", time_string);

    readParamFile(time_info);

    // Creat store json file
    if(fopen_s(&leapJsonFile, store_file_name, "w") != 0){
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
