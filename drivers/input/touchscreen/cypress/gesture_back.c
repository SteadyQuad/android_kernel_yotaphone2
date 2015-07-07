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

#define DEBUG

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/init.h>
#include <linux/device.h>
#include "gesture_back.h"
#include "gesture_front.h"

#define MULTI_DIR_SWIPE_SCROLL

/*Start : back gestures related changes */
int bsFilterEnabled = 0;

static gesture_slot_t bslots[MAX_SLOTS_COUNT];
static rect_area_t bsArea;

touch_accumulator_t bs_tempTouchAcc, bs_tempTouchAcc2;
static touch_accumulator_t bs_inputTouches[MAX_SLOTS_RECOGNITION];

single_gesture_t bs_singleTouchGestures[SINGLE_TOUCH_MAX_GESTURES_COUNT];
int bs_singleTouchGesturesCount = 0;

gesture_t bs_Gestures[SINGLE_TOUCH_MAX_GESTURES_COUNT + DOUBLE_TOUCH_MAX_GESTURES_COUNT];
static int bs_gesturesCount = 0;

static gesture_input_event_t bs_inputBuff[GESTURE_INPUT_EVENT_BUFF_SIZE];
static int bsInputBuffCount = 0;

static global_status_t bs_globalStatus;
screen_orientation_t bs_orient_state;

static struct input_mt_slot bsSlot_events[MAX_SLOTS_COUNT];
static bool bsLastFilterResult = false;

static key_info_t bsKeyInfo;
static Gesture_st_point_t bs_Gespt;

static int bsCurrentSlot = 0;
static bool bsSlotChanges = false;

extern struct input_dev *gestureInputDev;
struct input_dev *bsVirtualTPInputDev = NULL;

/*end */

static touch_timer_t bs_long_touch_timer;
static touch_timer_t bs_dt_timer;
static touch_timer_t bs_join_timer;

struct timer_list bs_lock_event_timer;

unsigned int last_tracking_id = 0;

#define BUF_STR_SIZE (200)
static char buf_str[BUF_STR_SIZE];

bs_gesture_status_t bsStatus;
bs_gesture_t bs_gestures[BS_MAX_GESTURES_COUNT];
int bs_gestures_count;

/*
 * Titanium Functions
 */
static void bs_init_engine(void);
static bool bs_gesture_driver_filter(unsigned int type, unsigned int code, int value);
static bool bs_handle_key_main(void);
static bool bs_handle_recognized_gesture(void);
static bool bs_handle_touch(void);
static void bs_update_gesture_type(void);
static void bs_update_available_gestures(void);
static bool bs_is_inside_area(input_point_t *, rect_area_t *);
static bool bs_is_inside_extended_areas(input_point_t *);
static all_gesture_compare_result_t bs_Compare_all_gestures(compare_gesture_mode_t);
static long bs_compare_gestures(touch_accumulator_t *, single_gesture_t *, single_gesture_t *,
        compare_gesture_mode_t);
static void bs_add_new_point(int);
static int bs_get_valid_gestures_count(void);
static int bs_get_used_slots_count(void);
static void bs_long_touch_callback(unsigned long data);
static void bs_lock_event_callback(unsigned long data);

/*
 * Titanium Functions end
 */

/* platinum function remove it later*/
static int bs_handle_key(char key, int isPressed);
static void bs_ten_to_two(char x);
static int bs_get_direction1(char, char);
static void bs_get_touch_area(bs_gesture_touch_area_t *, char);
static char bs_get_center(char status);
static all_gesture_compare_result_t bs_compare_all_gestures(compare_gesture_mode_t);
//static char bs_convert_hw_key(unsigned int);

static bool bs_handle_dt(int keyId);
static void bs_dt_callback(unsigned long data);
static void bs_join_callback(unsigned long data);
static bool bs_is_it_tap(void);
static void bs_handle_swipe_scroll(direction_t dir);
static bool bs_is_single_tap_enabled(void);

/*Titanium Function definitions*/

void bs_init_driver(void) {
    bs_init_engine();
    //setup long touch timer
    setup_timer(&bs_long_touch_timer.timer, bs_long_touch_callback, 0);

    setup_timer(&bs_lock_event_timer, bs_lock_event_callback, 1);

    //setup double click timer
    setup_timer(&bs_dt_timer.timer, bs_dt_callback, 0);

    //setup join key events timer
    setup_timer(&bs_join_timer.timer, bs_join_callback, 0);

    /*bsStatus.dt_status.dt_state = BS_DT_STATE_START;
    bsStatus.dt_status.is_dt_enabled = true;
    bsStatus.is_scroll_enabled = false; // disable till sys/fs will be implemented for that feature*/
    bsStatus.fs_lock_enabled = true;
    bsStatus.bs_lock_enabled = true;
    bsStatus.bs_unlock_enabled = false;
    bsStatus.fs_unlock_enabled = true;
    bsStatus.sendLockEvent = true;
}
/*
 * Init back screen gesture recognition engine
 */
void bs_init_engine(void) {

    gesture_slot_t* bslot;
    int i;
    LOG_V("bs_init_engine\n");

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        bslot = &bslots[i];
        bslot->status = SLOT_STATUS_NOT_USED;
        bslot->trackingId = -1;
        bslot->changed = false;
    }

    bsArea.leftUpX = BS_ZONE_LEFT_TOP_X;
    bsArea.leftUpY = BS_ZONE_LEFT_TOP_Y;
    bsArea.rightBottomX = BS_ZONE_RIGHT_BOTTOM_X;
    bsArea.rightBottomY = BS_ZONE_RIGHT_BOTTOM_Y;

    /*bottomArea.leftUpX = BOTTOM_ZONE_LEFT_TOP_X;
    bottomArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y;
    bottomArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    bottomArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;*/

    for (i = 0; i < MAX_SLOTS_RECOGNITION; i++) {
        bs_inputTouches[i].pointsCount = 0;
    }

    bs_globalStatus.gestureType = GESTURE_TYPE_SINGLE_OR_DOUBLE;
    bs_globalStatus.compareResult = COMPARE_RESULT_NOT_ACTIVE;

    bs_orient_state = SCREEN_ORIENTATION_ROTATE_0;

   /* bsStatus.recognition_state = BS_GESTURE_STATE_NOT_ACTIVE;
    bsStatus.status = 0;
    bsStatus.prev_status = 0;
    bsStatus.start_status = 0;
    bsStatus.key_state = BS_KEYS_STATE_NOT_PRESSED;
    bsStatus.dir = BS_DIR_NOT_CHANGED;
    bsStatus.action = -1;
    bsStatus.delayedKeyUP = -1;
    bsStatus.current_input_part = -1;
    bsStatus.scroll_state = BS_SCROLL_STATE_START;*/
}

/*
 * Load back screen gestures
 */
void bs_load_gestures(void) {

    int i;
    LOG_V("loading gesture for bs \n");

	// left swipe
    bs_tempTouchAcc.pointsCount = 2;
    bs_tempTouchAcc.points[0].x = 100;
    bs_tempTouchAcc.points[0].y = 0;
    bs_tempTouchAcc.points[1].x = 0;
    bs_tempTouchAcc.points[1].y = 0;

    bs_singleTouchGestures[0].touchAccPointsCount = bs_tempTouchAcc.pointsCount;
    memcpy(bs_singleTouchGestures[0].touchAccPoints, bs_tempTouchAcc.points,\
        bs_tempTouchAcc.pointsCount * sizeof(input_point_t));

    LOG_V("left swipe\n");
    normalize_size(&bs_tempTouchAcc);
    normalize_spacing(&bs_singleTouchGestures[0], &bs_tempTouchAcc);
    normalize_center(&bs_singleTouchGestures[0]);
    // gesture constraints
    bs_singleTouchGestures[0].minDx = 72;//10%
    bs_singleTouchGestures[0].minDy = 0;
    bs_singleTouchGestures[0].maxDx = 0;
    bs_singleTouchGestures[0].maxDy = 0;
    bs_singleTouchGestures[0].startPointArea.leftUpX = 0;
    bs_singleTouchGestures[0].startPointArea.leftUpY = BS_ZONE_LEFT_TOP_Y;
    bs_singleTouchGestures[0].startPointArea.rightBottomX = BS_ZONE_RIGHT_BOTTOM_X;
    bs_singleTouchGestures[0].startPointArea.rightBottomY = BS_ZONE_RIGHT_BOTTOM_Y;
    bs_singleTouchGestures[0].minTime = 50;
    bs_singleTouchGestures[0].maxTime = 3000;
    bs_singleTouchGestures[0].endPointArea.leftUpX = 0;
    bs_singleTouchGestures[0].endPointArea.leftUpY = BS_ZONE_LEFT_TOP_Y-100;
    bs_singleTouchGestures[0].endPointArea.rightBottomX = BS_ZONE_RIGHT_BOTTOM_X;
    bs_singleTouchGestures[0].endPointArea.rightBottomY = BS_ZONE_RIGHT_BOTTOM_Y;
    bs_singleTouchGestures[0].keyId = KEY_BS_GESTURE_SWIPE_LEFT;
    bs_singleTouchGestures[0].needSendKeyDown = false;
    bs_singleTouchGestures[0].isLongTapGesture = false;
    bs_singleTouchGestures[0].isDetectOnTouchUp = true;


    // right swipe
    bs_tempTouchAcc.pointsCount = 2;
    bs_tempTouchAcc.points[0].x = 0;
    bs_tempTouchAcc.points[0].y = 0;
    bs_tempTouchAcc.points[1].x = 100;
    bs_tempTouchAcc.points[1].y = 0;

    bs_singleTouchGestures[1].touchAccPointsCount = bs_tempTouchAcc.pointsCount;
    memcpy(bs_singleTouchGestures[1].touchAccPoints, bs_tempTouchAcc.points,\
        bs_tempTouchAcc.pointsCount * sizeof(input_point_t));

     LOG_V("Right swipe\n");

    normalize_size(&bs_tempTouchAcc);
    normalize_spacing(&bs_singleTouchGestures[1], &bs_tempTouchAcc);
    normalize_center(&bs_singleTouchGestures[1]);
    // gesture constraints
    bs_singleTouchGestures[1].minDx = 72;//10%
    bs_singleTouchGestures[1].minDy = 0;
    bs_singleTouchGestures[1].maxDx = 0;
    bs_singleTouchGestures[1].maxDy = 0;
    bs_singleTouchGestures[1].startPointArea.leftUpX = BS_ZONE_LEFT_TOP_X;
    bs_singleTouchGestures[1].startPointArea.leftUpY = BS_ZONE_LEFT_TOP_Y;
    bs_singleTouchGestures[1].startPointArea.rightBottomX = BS_ZONE_RIGHT_BOTTOM_X;
    bs_singleTouchGestures[1].startPointArea.rightBottomY = BS_ZONE_RIGHT_BOTTOM_Y;
    bs_singleTouchGestures[1].minTime = 50;
    bs_singleTouchGestures[1].maxTime = 3000;
    bs_singleTouchGestures[1].endPointArea.leftUpX = 0;
    bs_singleTouchGestures[1].endPointArea.leftUpY = BS_ZONE_LEFT_TOP_Y - 100;
    bs_singleTouchGestures[1].endPointArea.rightBottomX = BS_ZONE_RIGHT_BOTTOM_X;
    bs_singleTouchGestures[1].endPointArea.rightBottomY = BS_ZONE_RIGHT_BOTTOM_Y;
    bs_singleTouchGestures[1].keyId = KEY_BS_GESTURE_SWIPE_RIGHT;
    bs_singleTouchGestures[1].needSendKeyDown = false;
    bs_singleTouchGestures[1].isLongTapGesture = false;
    bs_singleTouchGestures[1].isDetectOnTouchUp = true;


       // recent apps, long touch
    // gesture constraints
    bs_singleTouchGestures[2].minDx = 0;
    bs_singleTouchGestures[2].minDy = 0;
    bs_singleTouchGestures[2].maxDx = 50;
    bs_singleTouchGestures[2].maxDy = 50;
    bs_singleTouchGestures[2].startPointArea.leftUpX = BS_ZONE_LEFT_TOP_X;
    bs_singleTouchGestures[2].startPointArea.leftUpY = BS_ZONE_LEFT_TOP_Y;
    bs_singleTouchGestures[2].startPointArea.rightBottomX = BS_ZONE_RIGHT_BOTTOM_X;
    bs_singleTouchGestures[2].startPointArea.rightBottomY = BS_ZONE_RIGHT_BOTTOM_Y;
    bs_singleTouchGestures[2].minTime = 450;
    bs_singleTouchGestures[2].maxTime = 550;
    bs_singleTouchGestures[2].endPointArea.leftUpX = -1;
    bs_singleTouchGestures[2].endPointArea.leftUpY = -1;
    bs_singleTouchGestures[2].endPointArea.rightBottomX = -1;
    bs_singleTouchGestures[2].endPointArea.rightBottomY = -1;
    bs_singleTouchGestures[2].keyId = KEY_BS_GESTURE_LONG_TAP;
    bs_singleTouchGestures[2].needSendKeyDown = true;
    bs_singleTouchGestures[2].isLongTapGesture = true;
    bs_singleTouchGestures[2].isDetectOnTouchUp = false;


    // single tap , need for double tap
    bs_singleTouchGestures[3].minDx = 0;
    bs_singleTouchGestures[3].minDy = 0;
    bs_singleTouchGestures[3].maxDx = 50;
    bs_singleTouchGestures[3].maxDy = 50;
    bs_singleTouchGestures[3].startPointArea.leftUpX = BS_ZONE_LEFT_TOP_X;
    bs_singleTouchGestures[3].startPointArea.leftUpY = BS_ZONE_LEFT_TOP_Y;
    bs_singleTouchGestures[3].startPointArea.rightBottomX = BS_ZONE_RIGHT_BOTTOM_X;
    bs_singleTouchGestures[3].startPointArea.rightBottomY = BS_ZONE_RIGHT_BOTTOM_Y;
    bs_singleTouchGestures[3].minTime = 30;
    bs_singleTouchGestures[3].maxTime = 300;
    bs_singleTouchGestures[3].endPointArea.leftUpX = -1;
    bs_singleTouchGestures[3].endPointArea.leftUpY = -1;
    bs_singleTouchGestures[3].endPointArea.rightBottomX = -1;
    bs_singleTouchGestures[3].endPointArea.rightBottomY = -1;
    bs_singleTouchGestures[3].keyId = KEY_BS_GESTURE_SINGLE_TAP;
    bs_singleTouchGestures[3].needSendKeyDown = false;
    bs_singleTouchGestures[3].isLongTapGesture = false;
    bs_singleTouchGestures[3].isDetectOnTouchUp = true;

    bs_singleTouchGesturesCount = 4;


    //*************************************************************

    bs_gesturesCount = 0;
    for (i = 0; i < bs_singleTouchGesturesCount; i++,bs_gesturesCount++) {
        bs_Gestures[bs_gesturesCount].status = GESTURE_STATUS_VALID;
        bs_Gestures[bs_gesturesCount].singleGesture = &bs_singleTouchGestures[i];
        bs_Gestures[bs_gesturesCount].dtGesture = 0;
    }

/*// left swipe
    bs_gestures[0].key_id = KEY_BS_GESTURE_SWIPE_LEFT;
    bs_gestures[0].start_area.h_bit = 4;
    bs_gestures[0].start_area.l_bit = 0;
    bs_gestures[0].parts_count = 1;
    bs_gestures[0].parts[0].dir = BS_DIR_LEFT;
    bs_gestures[0].parts[0].min_size = 4;
    bs_gestures[0].parts[0].max_size = 8;
    bs_gestures[0].isLongTapGesture = false;
    bs_gestures[0].isTapGesture = false;
    bs_gestures[0].minTime = 50;//ms
    bs_gestures[0].maxTime = 3000; //ms
    bs_gestures[0].needSendKeyDown = false;
    bs_gestures[0].enabled = false;

//right swipe
    bs_gestures[1].key_id = KEY_BS_GESTURE_SWIPE_RIGHT;
    bs_gestures[1].start_area.h_bit = 7;
    bs_gestures[1].start_area.l_bit = 3;
    bs_gestures[1].parts_count = 1;
    bs_gestures[1].parts[0].dir = BS_DIR_RIGHT;
    bs_gestures[1].parts[0].min_size = 4;
    bs_gestures[1].parts[0].max_size = 8;
    bs_gestures[1].isLongTapGesture = false;
    bs_gestures[1].isTapGesture = false;
    bs_gestures[1].minTime = 50;//ms
    bs_gestures[1].maxTime = 3000; //ms
    bs_gestures[1].needSendKeyDown = false;
    bs_gestures[1].enabled = false;

    //long tap
    bs_gestures[2].key_id = KEY_BS_GESTURE_LONG_TAP;
    bs_gestures[2].start_area.h_bit = 7;
    bs_gestures[2].start_area.l_bit = 0;
    bs_gestures[2].parts_count = 1;
    bs_gestures[2].parts[0].dir = BS_DIR_NOT_CHANGED;
    bs_gestures[2].parts[0].min_size = 0;
    bs_gestures[2].parts[0].max_size = 2;
    bs_gestures[2].isLongTapGesture = true;
    bs_gestures[2].isTapGesture = false;
    bs_gestures[2].minTime = 450; //ms
    bs_gestures[2].maxTime = 550; //ms
    bs_gestures[2].needSendKeyDown = true;
    bs_gestures[2].enabled = false;

    //single tap
    bs_gestures[3].key_id = KEY_BS_GESTURE_SINGLE_TAP;
    bs_gestures[3].start_area.h_bit = 7;
    bs_gestures[3].start_area.l_bit = 0;
    bs_gestures[3].parts_count = 1;
    bs_gestures[3].parts[0].dir = BS_DIR_NOT_CHANGED;
    bs_gestures[3].parts[0].min_size = 0;
    bs_gestures[3].parts[0].max_size = BS_TAP_GESTURE_MAX_SIZE;
    bs_gestures[3].isLongTapGesture = false;
    bs_gestures[3].isTapGesture = true;
    bs_gestures[3].minTime = 30; //ms
    bs_gestures[3].maxTime = 300; //ms
    bs_gestures[3].needSendKeyDown = false;
    bs_gestures[3].enabled = false;


    //RLR
    bs_gestures[4].key_id = KEY_BS_GESTURE_RIGHT_LEFT_RIGHT;
    bs_gestures[4].start_area.h_bit = 4;
    bs_gestures[4].start_area.l_bit = 0;
    bs_gestures[4].parts_count = 2;
    bs_gestures[4].parts[0].dir = BS_DIR_LEFT;
    bs_gestures[4].parts[0].min_size = 4;
    bs_gestures[4].parts[0].max_size = 8;
    bs_gestures[4].parts[1].dir = BS_DIR_RIGHT;
    bs_gestures[4].parts[1].min_size = 3;
    bs_gestures[4].parts[1].max_size = 8;
    bs_gestures[4].isLongTapGesture = false;
    bs_gestures[4].isTapGesture = false;
    bs_gestures[4].minTime = 50; //ms
    bs_gestures[4].maxTime = 1550; //ms
    bs_gestures[4].needSendKeyDown = false;
    bs_gestures[4].enabled = false;

    //LRL
    bs_gestures[5].key_id = KEY_BS_GESTURE_LEFT_RIGHT_LEFT;
    bs_gestures[5].start_area.h_bit = 7;
    bs_gestures[5].start_area.l_bit = 4;
    bs_gestures[5].parts_count = 2;
    bs_gestures[5].parts[0].dir = BS_DIR_RIGHT;
    bs_gestures[5].parts[0].min_size = 4;
    bs_gestures[5].parts[0].max_size = 8;
    bs_gestures[5].parts[1].dir = BS_DIR_LEFT;
    bs_gestures[5].parts[1].min_size = 3;
    bs_gestures[5].parts[1].max_size = 8;
    bs_gestures[5].isLongTapGesture = false;
    bs_gestures[5].isTapGesture = false;
    bs_gestures[5].minTime = 50; //ms
    bs_gestures[5].maxTime = 1550; //ms
    bs_gestures[5].needSendKeyDown = false;
    bs_gestures[5].enabled = false;

    bs_gestures_count = 6;
    */
}

/*
 * Virtual input device init
 */
 int bs_virtual_input_device_init(void) {
    int error;

    LOG_V("virtual_ts_init");
    bsVirtualTPInputDev = input_allocate_device();
    if (!bsVirtualTPInputDev) {
        LOG_E("Failed to allocate memory\n");
        return -ENOMEM;

    }

    bsVirtualTPInputDev->name = "bs_virtual_ts";
    bsVirtualTPInputDev->id.bustype = BUS_VIRTUAL;
    bsVirtualTPInputDev->dev.parent = NULL;

    set_bit(EV_SYN, bsVirtualTPInputDev->evbit);
    set_bit(EV_KEY, bsVirtualTPInputDev->evbit);
    set_bit(EV_ABS, bsVirtualTPInputDev->evbit);

    //set_bit(BTN_TOUCH, bsVirtualTPInputDev->keybit);
    set_bit(INPUT_PROP_DIRECT, bsVirtualTPInputDev->propbit);

    /* For multi touch */
    input_mt_init_slots(bsVirtualTPInputDev, 10);

    input_set_abs_params(bsVirtualTPInputDev, ABS_MT_POSITION_X, 0, BS_ZONE_RIGHT_BOTTOM_X, 0, 0);
    input_set_abs_params(bsVirtualTPInputDev, ABS_MT_POSITION_Y,0 ,
            BS_ZONE_RIGHT_BOTTOM_Y, 0, 0);

    input_set_abs_params(bsVirtualTPInputDev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(bsVirtualTPInputDev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(bsVirtualTPInputDev, ABS_MT_PRESSURE, 0, 255, 0, 0);

    error = input_register_device(bsVirtualTPInputDev);
    if (error) {
        input_free_device(bsVirtualTPInputDev);
        bsVirtualTPInputDev = NULL;
        LOG_E("Failed to register device\n");
    }

    return error;
}

/*
 * Unregister virtual input device
 */
void bs_virtual_input_device_exit(void) {

    if (bsVirtualTPInputDev != NULL) {
        input_unregister_device(bsVirtualTPInputDev);
        bsVirtualTPInputDev = NULL;
    }
}


bool back_gesture_driver_event(unsigned int type, unsigned int code,
                                 int value) {

    bool bsFilterResult;
    int   bsValueOrient;

	//LOG_E("gesture_driver_event : input_handle=%s,input_dev=%s,input_handler=%s\n",handle->name,handle->dev->name,handle->handler->name);
    LOG_V("back_gesture_driver_event:type: %d, code: %d, value: %d\n",type, code, value);

    // Screen orientation changes
    bsValueOrient = value;
    if (bs_orient_state == SCREEN_ORIENTATION_ROTATE_180) {
	LOG_V("Gesture_driver : bs_orient_state == SCREEN_ORIENTATION_ROTATE_180\n");
        if (type == EV_ABS && code == ABS_MT_POSITION_X) {
		LOG_V("Gesture_driver : type == EV_ABS && code == ABS_MT_POSITION_X\n");
            bsValueOrient = BS_ZONE_RIGHT_BOTTOM_X - value;
        } else if (type == EV_ABS && code == ABS_MT_POSITION_Y) {
		LOG_V("Gesture_driver : type == EV_ABS && code == ABS_MT_POSITION_Y\n");
            bsValueOrient = BS_ZONE_RIGHT_BOTTOM_Y - value;
        }
    }


    bsFilterResult = bs_gesture_driver_filter(type, code, bsValueOrient);
    if (type == EV_SYN && code == SYN_REPORT) {
        LOG_V("Sync report, filterResult - %d, lastResult - %d\n", bsFilterResult, bsLastFilterResult);
        if (bsFilterResult) {
		LOG_V("Gesture_driver : bsFilterResult=true\n");
            if(bsLastFilterResult == false) {
                // we start filter, send touch up event
                int j, last_slot = -1;
		LOG_V("Gesture_driver : bsLastFilterResult == false\n");
                LOG_V("Start filtering!!!!send touch up event\n");
                for (j = 0; j < MAX_SLOTS_COUNT; j++) {
                    if (bslots[j].trackingId >= 0) {
                        last_slot = j;
                    }
                }
                if(last_slot == -1) {
                    LOG_V("Send touch up for recognized single finger gesture\n");
                    input_event(bsVirtualTPInputDev, EV_ABS, ABS_MT_TRACKING_ID, -1);
                    input_event(bsVirtualTPInputDev, EV_KEY, BTN_TOUCH, 0);
                    input_event(bsVirtualTPInputDev, EV_SYN, SYN_REPORT, 0);
                } else {
                    for (j = 0; j < MAX_SLOTS_COUNT; j++) {
                        // if slot is active send touch up
                        if (bslots[j].trackingId >= 0) {

			    LOG_V("Send touch up for slot - %d\n", j);
                            input_event(bsVirtualTPInputDev, EV_ABS, ABS_MT_SLOT, j);
                            input_event(bsVirtualTPInputDev, EV_ABS, ABS_MT_TRACKING_ID, -1);
                            if (j == last_slot) {
                                // send the BTN_TOUCH UP because original one will be filtered
				    LOG_V("send the BTN_TOUCH UP for slot - %d\n", j);
                                input_event(bsVirtualTPInputDev, EV_KEY, BTN_TOUCH, 0);
                            }

                            input_event(bsVirtualTPInputDev, EV_SYN, SYN_REPORT, 0);
                        }
                    }
                    // reset current slot to zero
		      LOG_V("reset current slot to zero\n");
                	input_event(bsVirtualTPInputDev, EV_ABS, ABS_MT_SLOT, 0);
                    input_event(bsVirtualTPInputDev, EV_SYN, SYN_REPORT, 0);
                }
            }
            //clear buffer
            LOG_V("bsLastFilterResult == true.So,clear buffer\n");
			bsInputBuffCount = 0;
        } else {
            int i;
			LOG_V("Gesture_driver : filterResult=false\n");
			LOG_V("send all stored events and sync event\n");
            // send all stored events and sync event
            if (bsVirtualTPInputDev != NULL) {
                for (i = 0; i < bsInputBuffCount; i++) {
                    input_event(bsVirtualTPInputDev, bs_inputBuff[i].type, bs_inputBuff[i].code,
                            bs_inputBuff[i].value);
                }
                input_event(bsVirtualTPInputDev, type, code, value);
                bsInputBuffCount = 0;
            }
        }
		LOG_V("Gesture_driver : update lastFilterResult =%d\n",bsFilterResult);
        bsLastFilterResult = bsFilterResult;
    } else {
	    LOG_V("others Event type = %d and code = %d\n",type,code);
        if (bsInputBuffCount < GESTURE_INPUT_EVENT_BUFF_SIZE) {
            bs_inputBuff[bsInputBuffCount].code = code;
            bs_inputBuff[bsInputBuffCount].type = type;
            bs_inputBuff[bsInputBuffCount].value = value;
            bsInputBuffCount++;
        } else {
            LOG_V("Error, input buffer overload\n");
        }
    }

    // Filter out all events since we connected only to original touchscreen
    if(bsFilterEnabled == 0) {
        return true;
    } else {
        return false;
    }
}

bool bs_gesture_driver_filter(unsigned int type, unsigned int code, int value) {

    struct timespec ts;
    struct timeval tv;
    ktime_get_ts(&ts);
    tv.tv_sec = ts.tv_sec;
    tv.tv_usec = ts.tv_nsec / NSEC_PER_USEC;

	LOG_V("bs_gesture_driver_filter callback function : type=%d,code=%d,value=%d\n",type,code,value);
    if (type == EV_ABS) {
	LOG_V("bs_gesture_driver_filter : type == EV_ABS\n");
        if (code == ABS_MT_SLOT) {
		LOG_V("gesture_driver_filter : code == ABS_MT_SLOT. Return False\n");
            bsCurrentSlot = value;
            return false;
        }

        if (bsCurrentSlot < 0 || bsCurrentSlot >= MAX_SLOTS_COUNT) {
            LOG_V("error, slot is incorrect\n");
        } else {
            gesture_slot_t* sl = &bslots[bsCurrentSlot];
	 LOG_V("bs_gesture_driver_filter : currentSlot =%d\n",bsCurrentSlot);
            if (input_is_mt_value(code)) {
                bsSlot_events[bsCurrentSlot].abs[code - ABS_MT_FIRST] = value;
            }
            sl->time = tv;

            switch (code) {
			LOG_V("bs_gesture_driver_filter : switch code = %d having value =%d\n",code,value);
            case ABS_MT_POSITION_X:
                sl->changed = true;
                sl->positionX = value;
                break;
            case ABS_MT_POSITION_Y:
                sl->changed = true;
                sl->positionY = value;
                break;
            case ABS_MT_TRACKING_ID:
                bsSlotChanges = true;
                if (value < 0) {
                    // The slot is no longer in use but it retains its previous contents,
                    // which may be reused for subsequent touches.
                    sl->status = SLOT_STATUS_TOUCH_UP;
                    sl->changed = true;
                } else {
                    sl->changed = true;
                    sl->trackingId = value;
                }
                break;
            }
        }
    }else if (type == EV_KEY && code != BTN_TOUCH) {
	LOG_V("gesture_driver_filter : type == EV_KEY && code != BTN_TOUCH\n");
        // store key info
        bsKeyInfo.code = code;
        bsKeyInfo.value = value;
        bsKeyInfo.changed = true;

    }else if (type == EV_SYN && code == SYN_REPORT) {
	LOG_V("gesture_driver_filter : type == EV_SYN && code == SYN_REPORT\n");
        bs_handle_key_main();
        // Don't load CPU when we know this is not gesture
        if (!bsSlotChanges) {
		LOG_V("gesture_driver_filter : slot is not changed\n");
            switch(bs_globalStatus.compareResult) {
			LOG_V("gesture_driver_filter : bs_globalStatus.compareResult=%d\n",bs_globalStatus.compareResult);
                case COMPARE_RESULT_NOT_GESTURE:return false;
                case COMPARE_RESULT_NOT_GESTURE_FILTERED:return true;
                default:break;
            }
        }
		LOG_V("gesture_driver_filter: slot_changes=%d ,update to false\n",bsSlotChanges);
        bsSlotChanges = false;
        return bs_handle_touch();
    }
    return false;

}
/*
 * Main key handler that called from gesture_driver
 */
bool bs_handle_key_main(void) {

    //char key;

    LOG_V("bs_handle_key_main\n");

    if (bsKeyInfo.changed == false) {
        LOG_V("key has not been changed\n");
        return false;
    }
    bsKeyInfo.changed = false;

    if (bs_handle_recognized_gesture() == true) {
        LOG_V("it is mcu recognized gesture\n");
        return true;
    }

#ifdef BS_FORWARD_RAW_KEY_EVENTS
	LOG_V("input_report_key for keyInfo.code=%d,keyInfo.value=%d\n",bsKeyInfo.code,bsKeyInfo.value);
    input_report_key(gestureInputDev, bsKeyInfo.code, bsKeyInfo.value);
    input_sync(gestureInputDev);
#endif

    LOG_V("it is ordinary key code\n");

    input_event(bsVirtualTPInputDev, EV_KEY, bsKeyInfo.code, bsKeyInfo.value);

   /*key = bs_convert_hw_key(keyInfo.code);
    if (key != 0) {
		LOG_V("key=%d calling bs_handle_key() \n",key);
        bs_handle_key(key, keyInfo.value);
    }*/
    return true;
}

/*
 * Handle MCU recognized gesture as key event
 */
bool bs_handle_recognized_gesture(void) {
    if (bsKeyInfo.code >= KEY_BS_GESTURE_MINIMUM && bsKeyInfo.code <= KEY_BS_GESTURE_MAXIMUM) {
        // it is recognized gesture key event, resend it to android
        LOG_V("MCU recognized gesture key event, resend.. keyInfo.code=%d,keyInfo.value=%d\n",bsKeyInfo.code, bsKeyInfo.value);
	  if (gestureInputDev != NULL) {
	     input_report_key(gestureInputDev, bsKeyInfo.code, bsKeyInfo.value);
            input_sync(gestureInputDev);
	     input_event(gestureInputDev, EV_ABS, ABS_MT_POSITION_X, bs_Gespt.x);
	     input_event(gestureInputDev, EV_ABS, ABS_MT_POSITION_Y, bs_Gespt.y);
	     input_sync(gestureInputDev);
	}

        return true;
    }
    return false;
}

/*
 * Main touch handle function, recognize gesture,
 * return filter result
 */
bool bs_handle_touch() {
    gesture_slot_t* slot;
    bool result = false;
    int i;
    bool newEvent = false;

    bs_update_gesture_type();
    bs_update_available_gestures();

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        slot = &bslots[i];
        if (slot->status == SLOT_STATUS_NOT_USED && slot->changed) {
        LOG_V("slot->status is SLOT_STATUS_NOT_USED,slot->changed=%d\n",slot->changed);
			// new touch
            LOG_V("new touch in  slot %d  X=%d, y=%d, tracking ID = %d\n", i, slot->positionX,
                    slot->positionY, slot->trackingId);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                bs_add_new_point(i);
            }
            slot->changed = 0;
            slot->status = SLOT_STATUS_NEW_TOUCH;
			LOG_V("update newEvent=true,slot->Status=SLOT_STATUS_NEW_TOUCH,slot->changed=0\n");
        } else if ((slot->status == SLOT_STATUS_NEW_TOUCH || slot->status == SLOT_STATUS_CHANGED)
                && slot->changed) {
            LOG_V("slot->status=%d,slot->changed=%d\n",slot->status,slot->changed);
			//change
            LOG_V("change in current slot %d  X=%d, y=%d, tracking ID = %d\n", i, slot->positionX,
                    slot->positionY, slot->trackingId);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                bs_add_new_point(i);
            }
            slot->status = SLOT_STATUS_CHANGED;
            slot->changed = 0;
			LOG_V("update newEvent=true,slot's Status=SLOT_STATUS_CHANGED,slot->changed=0\n");
        } else if (slot->status == SLOT_STATUS_TOUCH_UP && slot->changed) {
            LOG_V("slot->status is SLOT_STATUS_TOUCH_UP,slot->changed=%d\n",slot->changed);
			// this is touch up
            LOG_V("touch up detected for slot - %d \n", i);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                bs_add_new_point(i);
            }
            slot->status = SLOT_STATUS_NOT_USED;
            slot->trackingId = -1;
            slot->changed = false;
			LOG_V("update newEvent=true,slot's Status=SLOT_STATUS_NOT_USED,slot->changed=0,slot->trackingId=-1\n");
        }

    }

    if (!newEvent) {
        // return if don't have new events
        LOG_V("newEvent=flase,No new events, bs_globalStatus.compareResult - %d\n", bs_globalStatus.compareResult);
        switch (bs_globalStatus.compareResult) {
        case COMPARE_RESULT_NOT_GESTURE:
        case COMPARE_RESULT_NOT_ACTIVE:
        case COMPARE_RESULT_NOT_RECOGNIZED:
            return false;
        case COMPARE_RESULT_NOT_GESTURE_FILTERED:
        case COMPARE_RESULT_GESTURE:
        case COMPARE_RESULT_PART_OF_GESTURE:
            return true;
        }
    }
    bs_update_gesture_type();
// ***********************************
//   Long touch support
    if (bs_globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
            && bslots[0].status == SLOT_STATUS_NEW_TOUCH) {
        // one new touch start long touch timer
        int ret;
		LOG_V("bs Long touch support : one new touch start long touch timer\n");
		LOG_V("gestureType is GESTURE_TYPE_SINGLE_OR_DOUBLE and slots[0].status is SLOT_STATUS_NEW_TOUCH\n");
        ret = mod_timer(&bs_long_touch_timer.timer, jiffies + msecs_to_jiffies(BS_LONG_TOUCH_MIN_TIME));
        bs_long_touch_timer.trackingId = bslots[0].trackingId;
        bs_long_touch_timer.count = 0;
    } else if (bs_globalStatus.gestureType != GESTURE_TYPE_SINGLE_OR_DOUBLE) {
        // //reset timer if more than one touch or touch up happened
	LOG_V("gestureType is not GESTURE_TYPE_SINGLE_OR_DOUBLE\n");
        bs_long_touch_timer.trackingId = -1;
    }

// Long touch support
//*****************************************

    switch (bs_globalStatus.gestureType) {
	LOG_V("bs_globalStatus.gestureType=%d\n",bs_globalStatus.gestureType);
    case GESTURE_TYPE_SINGLE_OR_DOUBLE:
    case GESTURE_TYPE_SINGLE:
    case GESTURE_TYPE_DOUBLE:
	LOG_V("bs_globalStatus.gestureType is GESTURE_TYPE_SINGLE_OR_DOUBLE||GESTURE_TYPE_SINGLE||GESTURE_TYPE_DOUBLE\n");
        switch (bs_globalStatus.compareResult) {
		LOG_V("bs_globalStatus.compareResult=%d\n",bs_globalStatus.compareResult);
        case COMPARE_RESULT_GESTURE:
            // notify and filter it
		    result = true;
		LOG_V("bs_globalStatus.compareResult is COMPARE_RESULT_GESTURE\n");
            if (bs_globalStatus.action < 0) {
                // already sent
                break;
            }
            if (bs_globalStatus.needSendKeyDown) {
				LOG_V("bs_globalStatus.needSendKeyDown==1\n");
                bs_globalStatus.delayedKeyUP = bs_globalStatus.action;
                LOG_V("send single key down\n");
                report_key_down(bs_globalStatus.action);
                bs_globalStatus.action = -1;
            }
			LOG_V("result is true\n");
            break;
        case COMPARE_RESULT_NOT_GESTURE:
        case COMPARE_RESULT_NOT_ACTIVE:
        case COMPARE_RESULT_NOT_RECOGNIZED:
            result = false;
            break;
        case COMPARE_RESULT_NOT_GESTURE_FILTERED:
            result = true;
			LOG_V("bs_globalStatus.compareResult is COMPARE_RESULT_NOT_GESTURE_FILTERED.Result is true\n");
            break;
        case COMPARE_RESULT_PART_OF_GESTURE:
            // probably gesture
            // moved to screen area, disable double touch gestures
            if (bs_get_valid_gestures_count() == 0) {
                bs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE_FILTERED;
            }
            result = true;
			LOG_V("bs_globalStatus.compareResult is COMPARE_RESULT_PART_OF_GESTURE.Result is true\n");
            break;
        }
        break;
    case GESTURE_TYPE_NOT_GESTURE:
	LOG_V("bs_globalStatus.gestureType is GESTURE_TYPE_NOT_GESTURE\n");
        switch (bs_globalStatus.compareResult) {
		LOG_V("bs_globalStatus.compareResult=%d\n",bs_globalStatus.compareResult);
        case COMPARE_RESULT_PART_OF_GESTURE:
        case COMPARE_RESULT_GESTURE:
        case COMPARE_RESULT_NOT_GESTURE_FILTERED:
            result = true;
		LOG_V("result is true\n");
            break;
        default:
            result = false;
            break;
        }
        break;
    }

//***********************************************
// check Touch up
// 1. single touch gesture
    if (bs_globalStatus.compareResult == COMPARE_RESULT_GESTURE
            && (bs_globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
                    || bs_globalStatus.gestureType == GESTURE_TYPE_SINGLE)
            && bslots[0].status == SLOT_STATUS_NOT_USED) {
	LOG_V("compareResult:COMPARE_RESULT_GESTURE && gestureType:GESTURE_TYPE_SINGLE_OR_DOUBLE||GESTURE_TYPE_SINGLE,slot[0]status SLOT_STATUS_NOT_USED\n");
        if (bs_globalStatus.needSendKeyDown && bs_globalStatus.delayedKeyUP >= 0) {
            LOG_E("Single touch:needSendKeyDown=%d,delayedKeyUP=%d:send key up\n",bs_globalStatus.needSendKeyDown,bs_globalStatus.delayedKeyUP);
            report_key_up(bs_globalStatus.delayedKeyUP);
            bs_globalStatus.delayedKeyUP = -1;
        } else {
			LOG_E("Single touch:report_key(bs_globalStatus.action)\n");
            report_key(bs_globalStatus.action);
        }

    }

//2. double touch gesture
   /* if (bs_globalStatus.compareResult == COMPARE_RESULT_GESTURE && bs_globalStatus.delayedKeyUP >= 0
            && bs_globalStatus.gestureType == GESTURE_TYPE_DOUBLE
            && (slots[0].status == SLOT_STATUS_NOT_USED
                    || slots[1].status == SLOT_STATUS_NOT_USED)) {
	LOG_V("compareResult:COMPARE_RESULT_GESTURE && gestureType:GESTURE_TYPE_DOUBLE,slot[0] and slot[1]status SLOT_STATUS_NOT_USED\n");
        if (bs_globalStatus.needSendKeyDown && bs_globalStatus.delayedKeyUP >= 0) {
            LOG_V("Double touch :needSendKeyDown=%d,delayedKeyUP=%d:send key up\n",bs_globalStatus.needSendKeyDown,bs_globalStatus.delayedKeyUP);
            report_key_up(bs_globalStatus.delayedKeyUP);
            bs_globalStatus.delayedKeyUP = -1;
        } else {
			LOG_V("Double touch:report_key(bs_globalStatus.action)\n");
            report_key(bs_globalStatus.action);
        }
    }*/

//3. lock gesture: some garbage touches
    if (bsStatus.bs_lock_enabled && (bs_get_used_slots_count() >=3) && (bs_globalStatus.compareResult == COMPARE_RESULT_NOT_GESTURE)) {
        mod_timer(&bs_lock_event_timer, jiffies+msecs_to_jiffies(LOCK_EVENT_TIMER));
    }

    if (bsStatus.bs_unlock_enabled && (bs_get_used_slots_count() == 1)) {
        report_key(KEY_BS_GESTURE_UNLOCK);
        bsStatus.bs_unlock_enabled = false;          //send only 1 unlock event
    }

//*******************************************************************
// make all gestures available if there are no touches.
    if (bs_get_used_slots_count() == 0) {
	LOG_V("make all gestures available if there are no touches.updating bs_globalStatus\n");
        for (i = 0; i < bs_gesturesCount; i++) {
            bs_Gestures[i].status = GESTURE_STATUS_VALID;

        }
        bs_globalStatus.gestureType = GESTURE_TYPE_SINGLE_OR_DOUBLE;
        bs_globalStatus.compareResult = COMPARE_RESULT_NOT_ACTIVE;
        bs_globalStatus.action = -1;
        bs_globalStatus.delayedKeyUP = -1;
    }
//********************************************************************

    LOG_V("Return handletouch filter result - %d\n", result);
    LOG_V("compare_result = %d\n", bs_globalStatus.compareResult);
    LOG_V("gesture type = %d\n", bs_globalStatus.gestureType);
    return result;
}

/*
 * Return count of all valid gestures
 */
int bs_get_valid_gestures_count(void) {
    int i, count;
    count = 0;
    for (i = 0; i < bs_gesturesCount; i++) {
        if (bs_Gestures[i].status != GESTURE_STATUS_NOT_VALID) {
            count++;
        }
    }
    return count;
}

/*
 * Get currently used slots count
 */
int bs_get_used_slots_count(void) {
    int i;
    int slotsCount = 0;

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        if (bslots[i].status != SLOT_STATUS_NOT_USED) {
            slotsCount++;
        } else if (bslots[i].changed) {
            slotsCount++;
        }
    }
    return slotsCount;
}


/*
 * Update type of gesture that we will recognize
 */
void bs_update_gesture_type(void) {
    int i;
    gesture_slot_t * sl;
    int slotsCount = 0;
LOG_V("update_gesture_type()\n");
    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        sl = &bslots[i];
        if (sl->status != SLOT_STATUS_NOT_USED) {
            slotsCount++;
        } else if (sl->changed) {
            slotsCount++;
        }
    }
	LOG_V("Current gestureType=%d\n",bs_globalStatus.gestureType);
    switch (bs_globalStatus.compareResult) {
    case COMPARE_RESULT_NOT_GESTURE:
	LOG_V("it is not gesture\n");
	break;
    case COMPARE_RESULT_GESTURE:
        //don't change gesture type if we already recognized gesture
		LOG_V("don't change gesture type if we already recognized gesture\n");
        break;
    case COMPARE_RESULT_NOT_ACTIVE:
    case COMPARE_RESULT_NOT_RECOGNIZED:
        if (slotsCount == MAX_SLOTS_RECOGNITION) {
            bs_globalStatus.gestureType = GESTURE_TYPE_DOUBLE;
        } else if (slotsCount > MAX_SLOTS_RECOGNITION) {
            bs_globalStatus.gestureType = GESTURE_TYPE_NOT_GESTURE;
        }
        break;
    case COMPARE_RESULT_PART_OF_GESTURE:
        // we should have decision about gesture type here
        if ((bs_globalStatus.gestureType == GESTURE_TYPE_DOUBLE
                && slotsCount != MAX_SLOTS_RECOGNITION)
                || (bs_globalStatus.gestureType == GESTURE_TYPE_SINGLE && slotsCount != 1)) {
            bs_globalStatus.gestureType = GESTURE_TYPE_NOT_GESTURE;
        }
        break;

    case COMPARE_RESULT_NOT_GESTURE_FILTERED:
        break;
    }
	LOG_V("updated gestureType=%d\n",bs_globalStatus.gestureType);
}

/*
 * Update available gesture list based on current type
 */
void bs_update_available_gestures(void) {

    int i;
    bool cancelST = false;
    bool cancelDT = false;

    LOG_V("update_available_gestures\n");
    switch (bs_globalStatus.gestureType) {
    case GESTURE_TYPE_DOUBLE:
        cancelST = true;
        break;
    case GESTURE_TYPE_NOT_GESTURE:
        cancelST = true;
        cancelDT = true;
        break;
    case GESTURE_TYPE_SINGLE_OR_DOUBLE:
        break;
    case GESTURE_TYPE_SINGLE:
        cancelDT = true;
        break;
    }

    for (i = 0; i < bs_gesturesCount; i++) {
        if (cancelST && bs_Gestures[i].singleGesture != 0
                && bs_Gestures[i].status != GESTURE_STATUS_RECOGNIZED) {
            bs_Gestures[i].status = GESTURE_STATUS_NOT_VALID;

        } /*else if (cancelDT && bs_Gestures[i].dtGesture != 0
                && bs_Gestures[i].status != GESTURE_STATUS_RECOGNIZED) {
            bs_Gestures[i].status = GESTURE_STATUS_NOT_VALID;
        }*/
    }
}

/*
 * Add new point, call gesture recognition if necessary
 * Update gesture compare_result.
 */
void bs_add_new_point(int inputId) {

    gesture_slot_t * slot;
    slot = &bslots[inputId];
	LOG_V("bs_add_new_point() for inputid - %d\n", inputId);
    switch (bs_globalStatus.compareResult) {
	LOG_V("bs_globalStatus.compareResult - %d\n", bs_globalStatus.compareResult);
    case COMPARE_RESULT_GESTURE:
        // nothing to do
        break;
    case COMPARE_RESULT_NOT_GESTURE:
    case COMPARE_RESULT_NOT_GESTURE_FILTERED:
        // nothing to do
        break;
//****************************************************
    case COMPARE_RESULT_NOT_RECOGNIZED: // second new point
    case COMPARE_RESULT_NOT_ACTIVE: // first new point
    case COMPARE_RESULT_PART_OF_GESTURE: // cann't accept new point
        switch (slot->status) {
        case SLOT_STATUS_NOT_USED:
            if (bs_globalStatus.compareResult == COMPARE_RESULT_PART_OF_GESTURE) {
                // cann't accept new touch in this state
                bs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE_FILTERED;
		LOG_V("COMPARE_RESULT_PART_OF_GESTURE,can't accept new touch in this state, update to COMPARE_RESULT_NOT_GESTURE_FILTERED\n");
                break;
            }
            // add first point
			LOG_V("add first point\n");
            bs_inputTouches[inputId].pointsCount = 1;
            bs_inputTouches[inputId].points[0].x = slot->positionX;
            bs_inputTouches[inputId].points[0].y = slot->positionY;
            bs_inputTouches[inputId].startPoint.x = slot->positionX;
            bs_inputTouches[inputId].startPoint.y = slot->positionY;
            bs_inputTouches[inputId].endPoint = bs_inputTouches[inputId].startPoint;
            bs_inputTouches[inputId].minX = slot->positionX;
            bs_inputTouches[inputId].minY = slot->positionY;
            bs_inputTouches[inputId].maxX = bs_inputTouches[inputId].minX;
            bs_inputTouches[inputId].maxY = bs_inputTouches[inputId].minY;
            bs_inputTouches[inputId].startTime = slot->time;
            bs_inputTouches[inputId].endTime = slot->time;

		/*storing first starting point */
	     bs_Gespt.x = bs_inputTouches[inputId].startPoint.x;
	     bs_Gespt.y = bs_inputTouches[inputId].startPoint.y;
	     LOG_V("stored first starting point,  bs_Gespt.x =%lu,bs_Gespt.y =%lu\n",bs_Gespt.x,bs_Gespt.y);

            if (!bs_is_inside_extended_areas(&bs_inputTouches[inputId].startPoint)) {
                bs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
			LOG_V("update globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE\n");	
            } else {
                bs_globalStatus.compareResult = COMPARE_RESULT_NOT_RECOGNIZED;
				LOG_V("update globalStatus.compareResult = COMPARE_RESULT_NOT_RECOGNIZED\n");
            }

            break;
        case SLOT_STATUS_NEW_TOUCH:
        case SLOT_STATUS_CHANGED:
        case SLOT_STATUS_TOUCH_UP:
            // add new point
            if (bs_inputTouches[inputId].pointsCount >= TOUCH_ACC_MAX_POINTS_COUNT) {
                // too many points, break
                //TODO something
                LOG_V("Too many points...break\n");
                break;
            }
			LOG_V("add new point\n");
            bs_inputTouches[inputId].points[bs_inputTouches[inputId].pointsCount].x = slot->positionX;
            bs_inputTouches[inputId].points[bs_inputTouches[inputId].pointsCount].y = slot->positionY;
            bs_inputTouches[inputId].pointsCount++;
            bs_inputTouches[inputId].endTime = slot->time;
            bs_inputTouches[inputId].endPoint.x = slot->positionX;
            bs_inputTouches[inputId].endPoint.y = slot->positionY;

            // update min and max values
            if (slot->positionX < bs_inputTouches[inputId].minX) {
                bs_inputTouches[inputId].minX = slot->positionX;
            } else if (slot->positionX > bs_inputTouches[inputId].maxX) {
                bs_inputTouches[inputId].maxX = slot->positionX;
            }

            if (slot->positionY < bs_inputTouches[inputId].minY) {
                bs_inputTouches[inputId].minY = slot->positionY;
            } else if (slot->positionY > bs_inputTouches[inputId].maxY) {
                bs_inputTouches[inputId].maxY = slot->positionY;
            }

            if (slot->status == SLOT_STATUS_TOUCH_UP) {
                all_gesture_compare_result_t result;
				LOG_V("slot->status == SLOT_STATUS_TOUCH_UP\n");
                result = bs_Compare_all_gestures(COMPARE_GESTURE_ALL);
                if (result.keyId >= 0) {
                    bs_globalStatus.action = result.keyId;
                    bs_globalStatus.needSendKeyDown = result.needSendKeyDown;
                    bs_globalStatus.compareResult = COMPARE_RESULT_GESTURE;
		LOG_V("updated globalStatus.action=%d,globalStatus.needSendKeyDown=%d,compareResult=COMPARE_RESULT_GESTURE(%d)\n",bs_globalStatus.action,bs_globalStatus.needSendKeyDown,bs_globalStatus.compareResult);
                }

            }
            // check last point area
			LOG_V("check last point area\n");
            if (bs_is_inside_extended_areas(
                    &bs_inputTouches[inputId].points[bs_inputTouches[inputId].pointsCount - 1])) {
                // we are in extended area now
				LOG_V("we are in extended area now\n");
                if (!bs_is_inside_extended_areas(&bs_inputTouches[inputId].startPoint)) {
                    // started from screen area and moved to extended area, not a gesture
					LOG_V("started from screen area and moved to extended area, not a gesture\n");
                    bs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                }
            } else {
			LOG_V("we are not in extended area now\n");
                if (bs_is_inside_extended_areas(&bs_inputTouches[inputId].startPoint)) {
                    // started from extended areas and moved to screen area.
                    // need to check for gesture
                    LOG_V("started in ext areas and moved to screen area...need to check for gesture\n");
                    if (bs_globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE) {
                        LOG_V("gesturetype is singleorDoube...update it to single and disable double touch gesture\n");
						bs_globalStatus.gestureType = GESTURE_TYPE_SINGLE;
                        //bs_disable_dt_gesture();
                    }
                    LOG_V("Moved to screen , gesture type- %d\n", bs_globalStatus.gestureType);

                    if (bs_globalStatus.compareResult == COMPARE_RESULT_PART_OF_GESTURE) {
                        all_gesture_compare_result_t result;
                        result = bs_Compare_all_gestures(COMPARE_GESTURE_ALL_MOVING);
						LOG_V("globalStatus.compareResult == COMPARE_RESULT_PART_OF_GESTURE\n");
                        if (result.keyId >= 0) {
                            bs_globalStatus.action = result.keyId;
                            bs_globalStatus.needSendKeyDown = result.needSendKeyDown;
                            bs_globalStatus.compareResult = COMPARE_RESULT_GESTURE;
	 LOG_V("updated globalStatus.action=%d,globalStatus.needSendKeyDown=%d,compareResult=COMPARE_RESULT_GESTURE\n",bs_globalStatus.action,bs_globalStatus.needSendKeyDown);	
                        }
                    } else if (bs_globalStatus.compareResult == COMPARE_RESULT_NOT_RECOGNIZED) {
                        all_gesture_compare_result_t result;
			LOG_V("globalStatus.compareResult == COMPARE_RESULT_NOT_RECOGNIZED\n");
                        result = bs_Compare_all_gestures(COMPARE_GESTURE_AND_START_POINT);
                        if (result.keyId >= 0) {
                            bs_globalStatus.action = result.keyId;
                            bs_globalStatus.needSendKeyDown = result.needSendKeyDown;
                            bs_globalStatus.compareResult = COMPARE_RESULT_PART_OF_GESTURE;
                        LOG_V("updated globalStatus.action=%d,globalStatus.needSendKeyDown=%d,compareResult=COMPARE_RESULT_GESTURE\n",bs_globalStatus.action,bs_globalStatus.needSendKeyDown);
			} else {
						LOG_V("it is not gesture\n");
                            bs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                        }
                    }
                } else {
                    // started from screen area, it is not a gesture
					LOG_V("started from screen area, it is not a gesture\n");
                    bs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                }
            }
            break;
        }
        break;
    }
   // LOG_V("add new point result input0 - %d\n", bs_globalStatus.compareResult);
}

/*
 * Check if point is inside area
 */
bool bs_is_inside_area(input_point_t * p, rect_area_t * a) {
    return (p->x >= a->leftUpX && p->x <= a->rightBottomX && p->y >= a->leftUpY
        && p->y <= a->rightBottomY);
}

/*
 * Check if point is inside one of the extended areas.
 */
bool bs_is_inside_extended_areas(input_point_t * point) {
    return (!bs_is_inside_area(point, &bsArea));
}

void bs_long_touch_callback(unsigned long data) {

    LOG_V("bs_long_touch_callback \n");
    if (bs_long_touch_timer.trackingId < 0) {
        LOG_V("timer was reset\n");
        return;
    }
    if (bs_get_used_slots_count() == 1 && bs_long_touch_timer.trackingId == bslots[0].trackingId) {
        if (bs_is_inside_extended_areas(&bs_inputTouches[0].startPoint)) {
            struct timespec ts;
            all_gesture_compare_result_t result;
            // update end time
            ktime_get_ts(&ts);
            bs_inputTouches[0].endTime.tv_sec = ts.tv_sec;
            bs_inputTouches[0].endTime.tv_usec = ts.tv_nsec / NSEC_PER_USEC;
            result = bs_Compare_all_gestures(COMPARE_GESTURE_LONG_TOUCH);
            if (result.keyId >= 0) {
                bs_globalStatus.compareResult = COMPARE_RESULT_GESTURE;
                bs_globalStatus.needSendKeyDown = result.needSendKeyDown;
                bs_globalStatus.action = result.keyId;
			LOG_V("updated bs_globalStatus.action=%d,bs_globalStatus.needSendKeyDown=%d,compareResult=COMPARE_RESULT_GESTURE\n",bs_globalStatus.action,bs_globalStatus.needSendKeyDown);

                if (bs_globalStatus.needSendKeyDown) {
                    bs_globalStatus.delayedKeyUP = bs_globalStatus.action;
                    report_key_down(bs_globalStatus.action);
                    bs_globalStatus.action = -1;
		      LOG_V("report_key_down for long touch,Action=%d\n",bs_globalStatus.action);
                }
            } else {
                // increase timer
                if (bs_long_touch_timer.count < BS_LONG_TOUCH_MAX_COUNT) {
                    bs_long_touch_timer.count++;
                    mod_timer(&bs_long_touch_timer.timer,
                            jiffies + msecs_to_jiffies(BS_LONG_TOUCH_MIN_TIME));
                }
            }
        }
    }
}

void bs_lock_event_callback(unsigned long data)
{
    if (bsStatus.bs_lock_enabled && (bs_get_used_slots_count() >=3) && (bs_globalStatus.compareResult == COMPARE_RESULT_NOT_GESTURE)) {
        report_key(KEY_BS_GESTURE_LOCK);
    }
}

/*
 * Compare user input with all valid gestures
 */
all_gesture_compare_result_t bs_Compare_all_gestures(compare_gesture_mode_t mode) {

    single_gesture_t tempGesture, tempGesture2;
    all_gesture_compare_result_t cResult;
    bool separateKeyUPKeyDown = false;
    long result;
    int i;
    int maxGestureValue = 0;
    int maxGestureKeyId = -1;

    LOG_V("bs_Compare_all_gestures for mode =%d\n",mode);
    copy_touch_accumulator(&bs_inputTouches[0], &bs_tempTouchAcc);
    LOG_V("bs_Compare_all_gestures():calling normalize func for single gesture\n");
    normalize_size(&bs_tempTouchAcc);
    normalize_spacing(&tempGesture, &bs_tempTouchAcc);
    normalize_center(&tempGesture);

    if (bs_globalStatus.gestureType == GESTURE_TYPE_DOUBLE) {
        copy_touch_accumulator(&bs_inputTouches[1], &bs_tempTouchAcc2);
	LOG_V("bs_Compare_all_gestures():calling normalize func for double gesture\n");
        normalize_size(&bs_tempTouchAcc2);
        normalize_spacing(&tempGesture2, &bs_tempTouchAcc2);
        normalize_center(&tempGesture2);
    }

    for (i = 0; i < bs_gesturesCount; i++) {
        if (bs_Gestures[i].status == GESTURE_STATUS_VALID) {
            if (bs_Gestures[i].singleGesture != 0
                    && (bs_globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
                            || bs_globalStatus.gestureType == GESTURE_TYPE_SINGLE)) {
                result = bs_compare_gestures(&bs_tempTouchAcc, &tempGesture, bs_Gestures[i].singleGesture,
                        mode);
                if (result == GESTURE_COMPARE_NOT_GESTURE) {
                    bs_Gestures[i].status = GESTURE_STATUS_NOT_VALID;
                    continue;
                }
                if (result > maxGestureValue) {
                    maxGestureValue = result;
                    maxGestureKeyId = bs_Gestures[i].singleGesture->keyId;
                    separateKeyUPKeyDown = bs_Gestures[i].singleGesture->needSendKeyDown;
                }
            }

            /*if (bs_Gestures[i].dtGesture != 0 && bs_globalStatus.gestureType == GESTURE_TYPE_DOUBLE) {
                result = bs_compare_dt_gestures(&bs_tempTouchAcc, &bs_tempTouchAcc2, &tempGesture,
                        &tempGesture2, bs_Gestures[i].dtGesture, mode);
                if (result > maxGestureValue) {
                    maxGestureValue = result;
                    maxGestureKeyId = bs_Gestures[i].dtGesture->keyId;
                    separateKeyUPKeyDown = bs_Gestures[i].dtGesture->gesture1.needSendKeyDown;
                }
            }*/
        }
    }
    LOG_V("Compare all gesture: max result - %d, key ID - %d\n", maxGestureValue, maxGestureKeyId);
    if (maxGestureValue > GESTURE_RECOGNITION_THRESHOLD) {
        // gesture is recognized
   		cResult.keyId = maxGestureKeyId;
        cResult.needSendKeyDown = separateKeyUPKeyDown;
		LOG_V("gesture is recognized, updated keyid=%d and needSendKeyDown=%d\n", cResult.keyId, cResult.needSendKeyDown);
    } else {
        cResult.keyId = -1;
        cResult.needSendKeyDown = false;
    }
    return cResult;
}


/*
 * Compare user input with gesture
 */
long bs_compare_gestures(touch_accumulator_t * gesture1, single_gesture_t * inputGesture,
        single_gesture_t * gesture2, compare_gesture_mode_t mode) {
    int deltaX, deltaY;
    unsigned long gestureTime;
    long result;

    LOG_V("Compare gesture with action ID - %d\n", gesture2->keyId);

    if (!gesture2->isLongTapGesture && mode == COMPARE_GESTURE_LONG_TOUCH){
	LOG_V("it is COMPARE_GESTURE_LONG_TOUCH!!!\n");
        return 0;
	}

    if(gesture2->isDetectOnTouchUp && mode == COMPARE_GESTURE_ALL_MOVING){
        LOG_V("Don't recognize gesture, it is not touch up yet!!!\n");
        return 0;
    }

    if (!bs_is_inside_area(&gesture1->startPoint, &gesture2->startPointArea)) {
        LOG_V("Start point out of gesture start area, x- %lu,  y- %lu\n", gesture1->startPoint.x,
                gesture1->startPoint.y);
		LOG_V("Recognize gesture's start point is not inside area of defined gestures,Return GESTURE_COMPARE_NOT_GESTURE !!\n");
        return GESTURE_COMPARE_NOT_GESTURE;
    }

    if (mode == COMPARE_GESTURE_ALL || mode == COMPARE_GESTURE_LONG_TOUCH
            || mode == COMPARE_GESTURE_ALL_MOVING) {
		LOG_V("mode=%d\n",mode);
        if (gesture2->endPointArea.leftUpX != -1) {
            if (!bs_is_inside_area(&gesture1->endPoint, &gesture2->endPointArea)) {
                LOG_V("end point outside end point area\n");
                return 0;
            }
        }

        deltaX = abs(gesture1->maxX - gesture1->minX);
        if (gesture2->minDx > 0 && gesture2->minDx > deltaX) {
            LOG_V("DeltaX is to small : return 0\n");
            return 0;
        }

        if (gesture2->maxDx > 0 && gesture2->maxDx < deltaX) {
            LOG_V("DeltaX is to big : return GESTURE_COMPARE_NOT_GESTURE\n");
            return GESTURE_COMPARE_NOT_GESTURE;
        }

        deltaY = abs(gesture1->maxY - gesture1->minY);
        if (gesture2->minDy > 0 && gesture2->minDy > deltaY) {
            LOG_V("DeltaY is to small : return 0\n");
            return 0;
        }

        if (gesture2->maxDy > 0 && gesture2->maxDy < deltaY) {
            LOG_V("DeltaY is to big : return GESTURE_COMPARE_NOT_GESTURE \n");
            return GESTURE_COMPARE_NOT_GESTURE;
        }

        gestureTime = tv2ms(&gesture1->endTime) - tv2ms(&gesture1->startTime);
        if (gesture2->minTime > 0 && gesture2->minTime > gestureTime) {
            LOG_V("Gesture time too small: gesture time - %lu, reference min value - %d\n",
                    gestureTime, gesture2->minTime);
            return 0;
        }

        if (gesture2->maxTime > 0 && gesture2->maxTime < gestureTime) {
            LOG_V("Gesture time too big: gesture time - %lu, reference max value - %d return GESTURE_COMPARE_NOT_GESTURE\n",
                    gestureTime, gesture2->maxTime);
            return GESTURE_COMPARE_NOT_GESTURE;
        }

    }
    if (gesture2->touchAccPointsCount == 0) {
        if (mode == COMPARE_GESTURE_LONG_TOUCH || mode == COMPARE_GESTURE_ALL) {
            LOG_V("Don't check template, It is a gesture : return 100\n");
            return 100;
        }
		LOG_V("gesture2->touchAccPointsCount == 0, return 0\n");
        return 0;
    }

    result = match(inputGesture, gesture2);
    LOG_V("compare result - %lu--> return result\n", result);
    return result;
}



/*  Platinum back function (remove it )*/

/*
 * Print byte in binary format
 */
void bs_ten_to_two(char x) {
    int i = 0;
    char str[9];
    for (i = 0; i < 8; i++) {
        if ((1 << (8 - i - 1)) & x)
            str[i] = '1';
        else
            str[i] = '0';
    }
    str[8] = '\0';
    LOG_V("status = %s\n", str);
}

/*
 * Get high and low bit of touch area
 */
void bs_get_touch_area(bs_gesture_touch_area_t * area, char status) {
    char h_bit;
    char l_bit = 7;
    char i;
    if (area == 0)
        return;
    h_bit = 0;
    l_bit = 7;
    if (status == 0) {
        h_bit = 0;
        l_bit = 0;
    } else {
        for (i = 0; i < 8; i++) {
            if ((1 << i) & status) {
                if (i > h_bit)
                    h_bit = i;
                if (i < l_bit)
                    l_bit = i;
            }
        }
    }
    area->h_bit = h_bit;
    area->l_bit = l_bit;
}


/*
 * Get center of touch area
 */
char bs_get_center(char status) {
    bs_gesture_touch_area_t m;
    char delta;
    char center;
    if (status == 0)
        return -1;
    bs_get_touch_area(&m, status);
    delta = m.h_bit - m.l_bit;
    center = m.l_bit + delta / 2;
    return center;
}

/*
 * Get movement direction based on old and new touch center
 */
int bs_get_direction1(char center_old, char center_new) {
    if (center_old == -1 || center_new == -1)
        return BS_DIR_NOT_CHANGED;
    if (center_old - center_new >= 3)
        return BS_DIR_RIGHT;
    if (center_new - center_old >= 3)
        return BS_DIR_LEFT;

    return BS_DIR_NOT_CHANGED;
}

/*
 * Handle key event
 */
int bs_handle_key(char key, int isPressed) {
    int dir;
    char old_status;
    struct timespec ts;
    struct timeval tv;
    int ret;
    int i;
    all_gesture_compare_result_t result;
    bs_gesture_touch_area_t m;
    unsigned int join_timer_delay;

    LOG_V("bs_handle_key, key - %d, ispressed=%d,state - %d\n", key,isPressed,bsStatus.key_state);
    // get current time
    ktime_get_ts(&ts);
    tv.tv_sec = ts.tv_sec;
    tv.tv_usec = ts.tv_nsec / NSEC_PER_USEC;
    LOG_V("time handle key: ms - %lu\n", tv2ms(&tv));

    switch (bsStatus.key_state) {
    case BS_KEYS_STATE_NOT_PRESSED:
        // should be first touch
        if (isPressed != 1) {
            // something wrong, should be 1 - true
            LOG_V("something wrong, should be 1 - true\n");
            return 0;
        }
        LOG_V("BS_KEYS_STATE_NOT_PRESSED : change key_state to DOWN\n");

        bs_join_timer.trackingId = -1;
        bsStatus.status |= key;
        bsStatus.prev_status = bsStatus.status;
        // change state to DOWN
        bsStatus.key_state = BS_KEYS_STATE_DOWN;
        bsStatus.recognition_state = BS_GESTURE_STATE_NOT_RECOGNIZED;
        bsStatus.start_status = bsStatus.status;

        bsStatus.startTime = tv;

        bsStatus.tracking_id = last_tracking_id;
        last_tracking_id++;

        bs_ten_to_two(bsStatus.status);

        // add 1-st input part
		LOG_V("add 1-st input part\n");
        bsStatus.input_parts[0].status_start = bsStatus.start_status;
        bsStatus.input_parts[0].status_end = bsStatus.start_status;
        bsStatus.input_parts[0].dir = BS_DIR_NOT_CHANGED;
        bsStatus.current_input_part = 0;


        // one new touch start long touch timer
        ret = mod_timer(&bs_long_touch_timer.timer, jiffies + msecs_to_jiffies(BS_LONG_TOUCH_TIME));
        bs_long_touch_timer.trackingId = bsStatus.tracking_id;

        // reset dc timer
        bs_dt_timer.trackingId = -1;
        break;
    case BS_KEYS_STATE_DOWN:
    case BS_KEYS_STATE_CHANGED:
        // move finger
        bs_join_timer.trackingId = -1;
        old_status = bsStatus.status;

        // set key to current mask
        if (isPressed == 1) {
            bsStatus.status |= key;
        } else {
            bsStatus.status &= ~key;
        }
        bsStatus.key_state = BS_KEYS_STATE_CHANGED;
        bs_ten_to_two(bsStatus.status);
        if (bsStatus.status == 0) {
            // this is up
            if (bs_is_it_tap()) {
                LOG_V("this is TAP, combined up state\n");
                bsStatus.key_state = BS_KEYS_STATE_UP;
                bsStatus.prev_status = old_status;
                bsStatus.input_parts[bsStatus.current_input_part].status_end = old_status;
                bsStatus.endTime = tv;
                return bs_handle_key(0, 0);
            } else {
                // start joint timer
                join_timer_delay = bsStatus.recognition_state == BS_GESTURE_STATE_SCROLL ?
                        BS_SWIPE_SCROLL_JOIN_TIMER_DELAY : BS_JOIN_TIMER_DELAY;
                ret = mod_timer(&bs_join_timer.timer, jiffies
                        + msecs_to_jiffies(join_timer_delay));
                bs_join_timer.trackingId = bsStatus.tracking_id;
                bs_long_touch_timer.trackingId = -1;
                bsStatus.endTime = tv;
                break;
            }

        }

        if (bsStatus.recognition_state == BS_GESTURE_STATE_NOT_GESTURE) {
            LOG_V("Down or changed state and it is not gesture\n");
            return 0;
        }

        // check how many keys are pressed
        bs_get_touch_area(&m, bsStatus.status);
        if((m.h_bit - m.l_bit + 1 ) >= BS_GESTURE_MAXIMUM_KEY_COUNT) {
            LOG_V("Too many pressed keys\n");
            bsStatus.recognition_state = BS_GESTURE_STATE_NOT_GESTURE;
            bsStatus.dt_status.dt_state = BS_DT_STATE_START;
            return 0;
        }

        dir = bs_get_direction1(bs_get_center(bsStatus.prev_status),
                bs_get_center(bsStatus.status));
		LOG_V("bsStatus.dir=%d and updated bs_get_direction1()=%d\n",bsStatus.dir,dir);
        switch(bsStatus.dir) {
        case BS_DIR_NOT_CHANGED:
            switch(dir){
            case BS_DIR_NOT_CHANGED:
                //nothing to do
                break;
            case BS_DIR_RIGHT:
            case BS_DIR_LEFT:
                // direction is defined first time
                bsStatus.dir = dir;
                bsStatus.input_parts[bsStatus.current_input_part].dir = dir;
                bsStatus.prev_status = bsStatus.status;
                bs_handle_swipe_scroll(dir);
                break;
            }
            break;
        case BS_DIR_RIGHT:
        case BS_DIR_LEFT:
            switch(dir){
            case BS_DIR_NOT_CHANGED:
                //nothing to do
                break;
            case BS_DIR_RIGHT:
            case BS_DIR_LEFT:
                if (bsStatus.dir != dir && bsStatus.recognition_state != BS_GESTURE_STATE_SCROLL) {
                   // user changed direction
				   LOG_V("user changed direction\n");
                   bsStatus.current_input_part++;
                   if(bsStatus.current_input_part >= BS_INPUT_PARTS_MAX_COUNT) {
                       bsStatus.recognition_state = BS_GESTURE_STATE_NOT_GESTURE;
                       LOG_V("It's not a gesture, too many parts...\n");
                       bsStatus.current_input_part = BS_INPUT_PARTS_MAX_COUNT-1;
                       bsStatus.dt_status.dt_state = BS_DT_STATE_START;
                   } else {
                       LOG_V("dir is changed\n");
                       bsStatus.input_parts[bsStatus.current_input_part].status_start =
                               bsStatus.input_parts[bsStatus.current_input_part-1].status_end;
                       bsStatus.input_parts[bsStatus.current_input_part].status_end =
                               bsStatus.input_parts[bsStatus.current_input_part].status_start;
                       bsStatus.input_parts[bsStatus.current_input_part].dir = dir;
                       bsStatus.dir = dir;
                   }

                }
                bs_handle_swipe_scroll(dir);
                bsStatus.prev_status = bsStatus.status;
                break;
            }
            break;
        }


        // update status_end for current part

        if(bsStatus.current_input_part >=0) {
            switch( bsStatus.input_parts[bsStatus.current_input_part].dir) {
                case BS_DIR_RIGHT:
                    if(bs_get_center(bsStatus.input_parts[bsStatus.current_input_part].status_end) >
                            bs_get_center(bsStatus.status)) {
                        LOG_V("BS_DIR_RIGHT - update status end \n");
                        bsStatus.input_parts[bsStatus.current_input_part].status_end =
                                bsStatus.status;
                    }
                    break;
                case BS_DIR_LEFT:
                    if(bs_get_center(bsStatus.input_parts[bsStatus.current_input_part].status_end) <
                            bs_get_center(bsStatus.status)) {
                        LOG_V("BS_DIR_LEFT - update status end \n");
                        bsStatus.input_parts[bsStatus.current_input_part].status_end =
                                bsStatus.status;
                    }
                    break;
                case BS_DIR_NOT_CHANGED:
                    bsStatus.input_parts[bsStatus.current_input_part].status_end = bsStatus.status;
                break;
            }
        }
        break;
    case BS_KEYS_STATE_UP:
        // release finger
        // reset long touch timer
        LOG_V("release finger,reset long touch timer:combined up state\n");
        bs_long_touch_timer.trackingId = -1;

        // log all parts
        LOG_V("current part - %d\n", bsStatus.current_input_part);
        for(i=0; i < bsStatus.current_input_part+1; i++) {
            LOG_V("part - %d, start - %d,  end - %d, dir - %d\n", i,
                    bs_get_center(bsStatus.input_parts[i].status_start),
                    bs_get_center(bsStatus.input_parts[i].status_end),
                    bsStatus.input_parts[i].dir);
        }
        LOG_V("recognition_state - %d\n", bsStatus.recognition_state);

        switch(bsStatus.recognition_state){
        case BS_GESTURE_STATE_GESTURE:
            // long tap was recognized
			LOG_V("long tap was recognized\n");
            if(bsStatus.action > 0){
                // need send key down/up
			LOG_V("report key down/up : bsStatus.action=%d\n",bsStatus.action);
                report_key(bsStatus.action);
            } else if(bsStatus.delayedKeyUP >0){
			    LOG_V("report key up : bsStatus.delayedKeyUP=%d\n",bsStatus.delayedKeyUP);
                report_key_up(bsStatus.delayedKeyUP);
            }
            break;
        case BS_GESTURE_STATE_NOT_GESTURE:
        case BS_GESTURE_STATE_SCROLL:
            // nothing to do
            break;
        case BS_GESTURE_STATE_NOT_RECOGNIZED:
        case BS_GESTURE_STATE_NOT_ACTIVE:
            result = bs_compare_all_gestures(COMPARE_GESTURE_ALL);
            if (bs_handle_dt(result.keyId) == false) {
                if ( result.keyId >= 0) {
                    LOG_V("report gesture with key ID - %d\n", result.keyId);
                    report_key(result.keyId);
                }
            }
            break;
        }
        bs_init_engine();
        break;
    }
    return 0;
}
/*
 * Compare user input with all back screen gestures
 */
all_gesture_compare_result_t bs_compare_all_gestures(compare_gesture_mode_t mode) {
    int i, j;
    char c_start;
    char c_end;
    char delta;
    bs_gesture_t * g;
    all_gesture_compare_result_t cResult;
    unsigned long gestureTime;
    bool needToContinue = false;
    int offset;

    LOG_V("bs_compare_all_gestures, mode - %d\n", mode);

    offset = 0;
    for (i = 0; i < bs_gestures_count; i++) {
        g = &bs_gestures[i];
        offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "G%d - ", g->key_id);

        if (!g->enabled
                && !(g->key_id == KEY_BS_GESTURE_SINGLE_TAP && bsStatus.dt_status.is_dt_enabled)) {
            offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "OFF## ");
            continue;
        }

        // check only long tap gestures in this mode
        if (mode == COMPARE_GESTURE_LONG_TOUCH && g->isLongTapGesture == false) {
            offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "only LT##");
            continue;
        }

        if (mode != COMPARE_GESTURE_LONG_TOUCH && g->isLongTapGesture == true) {
            offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "NO LT##");
            continue;
        }

        if (g->parts_count != bsStatus.current_input_part + 1) {
            offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "Parts !=##");
            continue;
        }
        needToContinue = false;
        for (j = 0; j < bsStatus.current_input_part + 1; j++) {

            // check part directions
            if (bsStatus.input_parts[j].dir != g->parts[j].dir && g->isTapGesture == false
                    && g->isLongTapGesture == false) {
                offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "Wrong dir##");
                needToContinue = true;
                break;
            }

            // check part size
            c_start = bs_get_center(bsStatus.input_parts[j].status_start);
            c_end = bs_get_center(bsStatus.input_parts[j].status_end);

            delta = abs(c_start - c_end) + 1;

            if (delta > g->parts[j].max_size || delta < g->parts[j].min_size) {
                offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "Wrong size##");
                needToContinue = true;
                break;
            }

        }
        if (needToContinue) {
            continue;
        }


        // check start area
        c_start = bs_get_center(bsStatus.start_status);
        if (c_start > g->start_area.h_bit || c_start < g->start_area.l_bit) {
            offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "Wrong start##");
            continue;
        }

        // check gesture min time
        gestureTime = tv2ms(&bsStatus.endTime) - tv2ms(&bsStatus.startTime);
        if (g->minTime > 0 && g->minTime > gestureTime) {
            offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "T.small: %lu##", gestureTime);
            continue;
        }

        // check gesture max time
        if (g->maxTime > 0 && g->maxTime < gestureTime) {
            offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "T. big: %lu##", gestureTime);
            continue;
        }
        // all conditions were checked, it is gesture
		LOG_V("all conditions were checked, it is gesture\n");
        offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "Matched##");
        LOG_V("%s", buf_str);
        cResult.keyId = g->key_id;
        cResult.needSendKeyDown = g->needSendKeyDown;
		LOG_V("updated keyId=%d,needSendKeyDown=%d\n",cResult.keyId,cResult.needSendKeyDown);
        return cResult;
    }

    offset += snprintf(buf_str + offset, BUF_STR_SIZE - offset, "Gesture is not recognized##");
	LOG_V("Gesture is not recognized\n");
    LOG_V("%s", buf_str);
    cResult.keyId = -1;
    cResult.needSendKeyDown = false;
    return cResult;
}



/*
 * Convert key code to bit value
 */
/*char bs_convert_hw_key(unsigned int code) {
    switch (code) {
    case KEY_BS_KEY0:
        return KEY0;
    case KEY_BS_KEY1:
        return KEY1;
    case KEY_BS_KEY2:
        return KEY2;
    case KEY_BS_KEY3:
        return KEY3;
    case KEY_BS_KEY4:
        return KEY4;
    case KEY_BS_KEY5:
        return KEY5;
    case KEY_BS_KEY6:
        return KEY6;
    case KEY_BS_KEY7:
        return KEY7;
    }
    return 0;
}*/

/*void bs_long_touch_callback(unsigned long data) {

    all_gesture_compare_result_t result;
    struct timespec ts;
    struct timeval tv;

    LOG_V("bs long touch callback \n");

    if (bs_long_touch_timer.trackingId < 0) {
        LOG_V("bs timer was reset\n");
        return;
    }
    if (bsStatus.tracking_id != bs_long_touch_timer.trackingId) {
        LOG_V("timer was set for another key\n");
        return;
    }

    if (bsStatus.recognition_state == BS_GESTURE_STATE_NOT_GESTURE) {
        LOG_V("It is not a gesture, don't check for long tap\n");
        return;
    }

    // update end time
    ktime_get_ts(&ts);
    tv.tv_sec = ts.tv_sec;
    tv.tv_usec = ts.tv_nsec / NSEC_PER_USEC;
    bsStatus.endTime = tv;
    LOG_V("time bs_long_touch_callback: ms - %lu\n", tv2ms(&tv));
    result = bs_compare_all_gestures(COMPARE_GESTURE_LONG_TOUCH);
    if (result.keyId >= 0) {
        LOG_V("long tap  gesture with key ID - %d\n", result.keyId);
        bsStatus.recognition_state = BS_GESTURE_STATE_GESTURE;
        if (result.needSendKeyDown == true) {
            LOG_V("send key down for long tap\n");
            report_key_down(result.keyId);
            bsStatus.delayedKeyUP = result.keyId;
            bsStatus.action = -1;
        } else {
            LOG_V("send down up after key up\n");
            bsStatus.action = result.keyId;
            bsStatus.delayedKeyUP = -1;
        }
    }
}*/

static bool bs_handle_dt(int keyId) {
    int ret;
    char start, end, center;
    if (bsStatus.dt_status.is_dt_enabled == false) {
        // return if gesture is disabled
        LOG_V("bs_handle_dt - feature is disabled\n");
        return false;
    }

    LOG_V("bs_handle_dt: state - %d\n", bsStatus.dt_status.dt_state);

    switch (bsStatus.dt_status.dt_state) {
    case BS_DT_STATE_START:
        if (keyId != KEY_BS_GESTURE_SINGLE_TAP) {
            // it is not tap
            return false;
        }
        //start double tap timer
        ret = mod_timer(&bs_dt_timer.timer,
                jiffies + msecs_to_jiffies(BS_DC_TIME));
        bsStatus.dt_status.dt_state = BS_DT_STATE_FIRST_TAP;

        start = bs_get_center(bsStatus.input_parts[0].status_start);
        end = bs_get_center(bsStatus.input_parts[0].status_end);
        center = (start + end)/2;
        LOG_V("First TAP: start - %d, end - %d, center - %d\n", start, end, center);
        bsStatus.dt_status.first_tap_center = center;
        bs_dt_timer.trackingId = bsStatus.tracking_id;
        break;
    case BS_DT_STATE_FIRST_TAP:
        bsStatus.dt_status.dt_state = BS_DT_STATE_START;
        bs_dt_timer.trackingId = -1;

        if (keyId != KEY_BS_GESTURE_SINGLE_TAP) {
            LOG_V("second touch is not TAP, reset state and filter it out\n");
            break;
        }

        start = bs_get_center(bsStatus.input_parts[0].status_start);
        end = bs_get_center(bsStatus.input_parts[0].status_end);
        center = (start + end)/2;
        LOG_V("Second TAP: start - %d, end - %d, center - %d\n", start, end, center);
        if( abs(bsStatus.dt_status.first_tap_center - center) <= BS_DT_INTERVAL ) {
            LOG_V("it is a second tap , Report it\n");
            report_key(KEY_BS_GESTURE_DOUBLE_TAP);
        }
        break;
    }

    return true;
}

void bs_dt_callback(unsigned long data) {
    if (bs_dt_timer.trackingId == -1) {
        LOG_V("double click timer was reset\n");
        return;
    }

    if (bsStatus.dt_status.dt_state == BS_DT_STATE_FIRST_TAP) {
        //reset dc state
        bsStatus.dt_status.dt_state = BS_DT_STATE_START;
        if (bs_is_single_tap_enabled()) {
            LOG_V("DT cb send single tap\n");
            report_key(KEY_BS_GESTURE_SINGLE_TAP);
        }
    }
}

void bs_join_callback(unsigned long data) {

    if (bs_join_timer.trackingId == -1) {
        LOG_V("join timer was reset\n");
        return;
    }

    // it is touch up
    LOG_V("it is touch up : combined delayed up state\n");
    bsStatus.key_state = BS_KEYS_STATE_UP;
    bs_handle_key(0, 0);
    return;
}

bool bs_is_it_tap(void) {
    int i;
    char start, end, delta;

    LOG_V("bs_is_it_tap\n");
    for (i = 0; i < bsStatus.current_input_part + 1; i++) {
        LOG_V("part - %d, start - %d,  end - %d, dir - %d\n", i,
            bs_get_center(bsStatus.input_parts[i].status_start),
            bs_get_center(bsStatus.input_parts[i].status_end),
            bsStatus.input_parts[i].dir);
    }
    start = bs_get_center(bsStatus.input_parts[0].status_start);
    end = bs_get_center(bsStatus.input_parts[0].status_end);
    delta = abs(start - end) + 1;
    if (bsStatus.input_parts[0].dir == BS_DIR_NOT_CHANGED || delta <= BS_TAP_GESTURE_MAX_SIZE) {
        return true;
    } else {
        return false;
    }
}


static void bs_handle_swipe_scroll(direction_t dir) {
    LOG_V("bs_handle_swipe_scroll\n");
    if(dir == BS_DIR_NOT_CHANGED || bsStatus.is_scroll_enabled == false) {
        return;
    }

#ifdef MULTI_DIR_SWIPE_SCROLL
    if(dir == BS_DIR_LEFT) {
        LOG_V("scroll left\n");
        report_key(KEY_BS_GESTURE_SCROLL_LEFT);
    } else {
        LOG_V("scroll right\n");
        report_key(KEY_BS_GESTURE_SCROLL_RIGHT);
    }
    // If we detected scroll disable other gestures processing
    bsStatus.recognition_state = BS_GESTURE_STATE_SCROLL;
    return;
#else

    switch(bsStatus.scroll_state) {
        case BS_SCROLL_STATE_START:
            if(dir == BS_DIR_LEFT) {
                LOG_V("first scroll left\n");
                bsStatus.scroll_state = BS_SCROLL_STATE_LEFT;
                report_key(KEY_BS_GESTURE_SCROLL_LEFT);
            } else {
                LOG_V("first scroll right\n");
                bsStatus.scroll_state = BS_SCROLL_STATE_RIGHT;
                report_key(KEY_BS_GESTURE_SCROLL_RIGHT);
            }
            break;
        case BS_SCROLL_STATE_LEFT:
            if(dir == BS_DIR_LEFT) {
                LOG_V("new scroll left\n");
                report_key(KEY_BS_GESTURE_SCROLL_LEFT);
            } else {
                LOG_V("scroll - dir changed, not gesture\n");
                bsStatus.scroll_state = BS_SCROLL_STATE_NOT_GESTURE;
            }
            break;
        case  BS_SCROLL_STATE_RIGHT:
            if(dir == BS_DIR_RIGHT) {
                LOG_V("new scroll right\n");
                report_key(KEY_BS_GESTURE_SCROLL_RIGHT);
            } else {
                LOG_V("scroll - dir changed, not gesture\n");
                bsStatus.scroll_state = BS_SCROLL_STATE_NOT_GESTURE;
            }
            break;
        case BS_SCROLL_STATE_NOT_GESTURE:
            // nothing to do
            return;
    }
    // If we detected scroll disable other gestures processing
    bsStatus.recognition_state = BS_GESTURE_STATE_SCROLL;
#endif
}

static bool bs_is_single_tap_enabled(void) {
    int i;
    for (i = 0; i < bs_gestures_count; i++) {
        if (bs_gestures[i].key_id == KEY_BS_GESTURE_SINGLE_TAP) {
            return bs_gestures[i].enabled;
        }
    }
    return false;
}
