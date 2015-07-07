/*
 * Gesture recognition driver
 *
 * Copyright (c) 2013,2014 YotaDevices.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#ifndef GESTURE_FRONT_H_FILE
#define GESTURE_FRONT_H_FILE

#define TOUCH_ACC_MAX_POINTS_COUNT (200)
#define GESTURE_TEMPLATE_MAX_POINTS_COUNT (16)
#define POINTS_PER_GESTURE (32)
#define GESTURE_SCALE_FACTOR (500)
#define MAX_SLOTS_COUNT  (10)
#define SINGLE_TOUCH_MAX_GESTURES_COUNT (15)
#define DOUBLE_TOUCH_MAX_GESTURES_COUNT (1)
#define MAX_SLOTS_RECOGNITION (2)
#define GESTURE_RECOGNITION_THRESHOLD (90)

/* Extended zones definition  */
/* Tap extended area  */
#define TOP_ZONE_LEFT_TOP_X (0)
#define TOP_ZONE_LEFT_TOP_Y (0)
#define TOP_ZONE_RIGHT_BOTTOM_X (1080)
#define TOP_ZONE_RIGHT_BOTTOM_Y (70)

/* Bottom extended area  */
#define BOTTOM_ZONE_LEFT_TOP_X (0)
#define BOTTOM_ZONE_LEFT_TOP_Y (1920)
#define BOTTOM_ZONE_RIGHT_BOTTOM_X (1080)
#define BOTTOM_ZONE_RIGHT_BOTTOM_Y (1920)

/*capsense touch area */
#define CS_ZONE_LEFT_TOP_X (132)
#define CS_ZONE_LEFT_TOP_Y (0)
#define CS_ZONE_RIGHT_BOTTOM_X (947)
#define CS_ZONE_RIGHT_BOTTOM_Y (0)

/* Gesture time definition */
/* Pan gesture min/max time */
#define PAN_MIN_TIME (0)
#define PAN_MAX_TIME (3000)

//Gestures actions IDs - Temp!!!!
#define ACTION_SCREEN_OFF        KEY_FS_GESTURE_SCREEN_OFF
#define ACTION_BACK_PAN          KEY_BACK
#define ACTION_BACK_FLICK        KEY_BACK
#define ACTION_HOME_PAN          KEY_HOMEPAGE
#define ACTION_SEARCH_PAN        KEY_SEARCH
#define ACTION_SCREEN_ON_PAN     KEY_FS_GESTURE_SCREEN_ON
#define ACTION_SCREEN_ON_FLICK   KEY_FS_GESTURE_SCREEN_ON
#define ACTION_PUT_TO_BACK       KEY_FS_GESTURE_PUT_TO_BACK
#define ACTION_RECENT_APPS_LONG_TOUCH    KEY_FS_GESTURE_RECENT_APPS
#define ACTION_TOP_AREA_LONG_TOUCH       KEY_FS_GESTURE_TOP_AREA_LONG_TOUCH

/* Long tap timer values */
#define LONG_TOUCH_MIN_TIME (500)
#define LONG_TOUCH_MAX_COUNT (3)

/* Timer value to send LOCK event */
#define LOCK_EVENT_TIMER (200)

/* Return this value if user input is definitely not gesture */
#define GESTURE_COMPARE_NOT_GESTURE (-1)

/* Input event buffer size */
#define GESTURE_INPUT_EVENT_BUFF_SIZE (100)

/*Capsense double touch slots*/
#define DOUBLE_TOUCH_SLOT (1)
#define DOUBLE_SLOT 0x01

/* Log levels */
typedef enum log_level {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_DEBUG = 1,
    LOG_LEVEL_VERBOSE =2,
} log_level_t;

/* Gesture compare modes  */
typedef enum compare_gesture_mode {
    COMPARE_GESTURE_AND_START_POINT = 0,
    COMPARE_GESTURE_ALL = 1,
    COMPARE_GESTURE_LONG_TOUCH = 2,
    COMPARE_GESTURE_ALL_MOVING = 3,
} compare_gesture_mode_t;

/* Gesture compare states  */
typedef enum compare_result {
    COMPARE_RESULT_NOT_GESTURE = 0,
    COMPARE_RESULT_GESTURE = 1,
    COMPARE_RESULT_NOT_RECOGNIZED = 2,
    COMPARE_RESULT_PART_OF_GESTURE = 3,
    COMPARE_RESULT_NOT_ACTIVE = 4,
    COMPARE_RESULT_NOT_GESTURE_FILTERED = 5,
} compare_result_t;

/* Predefined gesture status depends on user input */
typedef enum gesture_status {
    GESTURE_STATUS_VALID = 0,
    GESTURE_STATUS_NOT_VALID = 1,
    GESTURE_STATUS_RECOGNIZED = 2,
} gesture_status_t;

/* Slot status */
typedef enum slot_status {
    SLOT_STATUS_NOT_USED = 0,
    SLOT_STATUS_NEW_TOUCH = 1,
    SLOT_STATUS_CHANGED = 2,
    SLOT_STATUS_TOUCH_UP = 3,
} slot_status_t;

/* Current gesture type that we are going to recognize*/
typedef enum gesture_type{
    GESTURE_TYPE_SINGLE_OR_DOUBLE = 0,
    GESTURE_TYPE_SINGLE = 1,
    GESTURE_TYPE_DOUBLE = 2,
    GESTURE_TYPE_NOT_GESTURE = 3,
} gesture_type_t;

/* Screen orientation */
typedef enum screen_orientation{
    SCREEN_ORIENTATION_ROTATE_0 = 0,
    SCREEN_ORIENTATION_ROTATE_180 = 1,
} screen_orientation_t;

/* Slot */
typedef struct gesture_slot {
    slot_status_t status;       // slot status
    bool changed;               // is there changes in slot
    int positionX;              // current x position
    int positionY;              // current y position
    int trackingId;             // current tracking Id
    struct timeval time;        // time of changes
} gesture_slot_t;

/*  Point */
typedef struct input_point {
    long x;     // x coordinate
    long y;     // y coordinate
} input_point_t;

/* Rectangle area */
typedef struct rect_area {
    int leftUpX;        // left up corner X
    int leftUpY;        // left up corner Y
    int rightBottomX;   // right bottom corner X
    int rightBottomY;   // right bottom corner Y
} rect_area_t;

/* Single gesture definition */
typedef struct single_gesture {
    input_point_t points[POINTS_PER_GESTURE];   // gesture template
    int pointsCount;                            // template point count
    rect_area_t startPointArea;                 // start point area
    rect_area_t endPointArea;                   // end point area
    int minTime;                                // min time value for gesture in ms
    int maxTime;                      // max time value for gesture in ms
    int minDx;                        // min x delta between max and min values
    int minDy;                        // min y delta between max and min values
    int maxDx;                        // max x delta between max and min values
    int maxDy;                        // max y delta between max and min values
    int keyId;                        // gesture key Id
    bool needSendKeyDown;             // do we need send key down at the gesture recognition moment
    bool isLongTapGesture;                // Is it tap gesture
    bool isDetectOnTouchUp;           // recognize gesture after touch up only
    //need to add following two fields in order to preserve input/output consistency
    int touchAccPointsCount;
    input_point_t touchAccPoints[GESTURE_TEMPLATE_MAX_POINTS_COUNT];
} single_gesture_t;

/* Double touch gesture definition */
typedef struct double_touch_gesture {
    single_gesture_t gesture1;            // gesture for first finger
    single_gesture_t gesture2;            // gesture for second finger
    int keyId;                            // gesture key Id
} double_touch_gesture_t;

/* Touch accumulator */
typedef struct touch_accumulator {
    input_point_t points[TOUCH_ACC_MAX_POINTS_COUNT];   // touch point
    int pointsCount;                                    // point count
    input_point_t startPoint;                           // start point of user input
    input_point_t endPoint;                             // end point of user input
    struct timeval startTime;                           // start time
    struct timeval endTime;                             // end time
    int minX;                                           // minimum x-coordinate value
    int maxX;                                           // maximum x-coordinate value
    int minY;                                           // minimum y-coordinate value
    int maxY;                                           // maximum y-coordinate value
} touch_accumulator_t;

typedef struct touch_timer {
    int trackingId;                   // touch tracking id
    struct timer_list timer;          // kernel timer struct
    int count;                        // timer counter, can restart timer if gesture not recognized
} touch_timer_t;

/* All gestures compare result */
typedef struct all_gesture_compare_result {
    int keyId;                        // key Id of recognized gesture
    bool needSendKeyDown;             // do we need send key down at the gesture recognition moment
} all_gesture_compare_result_t;

/* Gesture definition */
typedef struct gesture {
    gesture_status_t status;              // gesture status: valid, not_valid etc.
    single_gesture_t * singleGesture;     // pointer to single gesture
    double_touch_gesture_t * dtGesture;   // pointer to double touch gesture
} gesture_t;

/* Global recognition status */
typedef struct global_status {
    gesture_type_t gestureType;     // what type of gesture we try to recognize
    compare_result_t compareResult; // current gesture recognition result
    int action;                     // recognized gesture key Id
    bool needSendKeyDown;           // do we need send key down at the gesture recognition moment
    int delayedKeyUP;               // save key Id here after sent key down
} global_status_t;

/* Struct to pack input event before sending */
typedef struct gesture_input_event {
    unsigned int type;          // type of event
    unsigned int code;          // code of event
    int value;                  // event value
} gesture_input_event_t;

void report_key(unsigned int);
void report_key_down(unsigned int);
void report_key_up(unsigned int);
unsigned long int tv2ms(struct timeval *a);
void normalize_size(touch_accumulator_t *);
void normalize_spacing(single_gesture_t *, touch_accumulator_t *);
void normalize_center(single_gesture_t *);
void gesture_input_dev_fill(void);
void copy_touch_accumulator(touch_accumulator_t *, touch_accumulator_t *);
long match(single_gesture_t *, single_gesture_t *);


/* Runtime logging defines */
extern log_level_t log_level;

#ifdef DEBUG
#define LOG_V(fmt, args...) if(log_level == LOG_LEVEL_VERBOSE ) \
    pr_info(KBUILD_MODNAME ": " fmt, ## args)
#define LOG_D(fmt, args...) if(log_level >= LOG_LEVEL_DEBUG ) \
    pr_info(KBUILD_MODNAME ": " fmt, ## args)
#define LOG_E(fmt, args...) pr_err(KBUILD_MODNAME ": " fmt, ## args)
#else
#define LOG_V(fmt, args...)
#define LOG_D(fmt, args...)
#define LOG_E(fmt, args...)
#endif
#endif
