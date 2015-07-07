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
#include "gesture_front.h"
#include "gesture_back.h"
#include "gesture_config.h"



#define FS_INPUT_DRIVER_NAME "cyttsp5_mt_front"
#define BS_INPUT_DRIVER_NAME "cyttsp5_mt_back"
#define CAPS_INPUT_DRIVER_NAME "cyttsp3-i2c"

/*capsense*/
static int capsenseSlots = 0;
static gesture_slot_t  capsense_slots[MAX_SLOTS_COUNT];
static int CurrentCapsenseSlot = 0;
static bool CS_slot_changes = false;
static global_status_t cs_globalStatus;

touch_accumulator_t cs_tempTouchAcc, cs_tempTouchAcc2;
static touch_accumulator_t cs_inputTouches[MAX_SLOTS_RECOGNITION];
single_gesture_t cs_singleTouchGestures[SINGLE_TOUCH_MAX_GESTURES_COUNT];
int cs_singleTouchGesturesCount = 0;

gesture_t cs_Gestures[SINGLE_TOUCH_MAX_GESTURES_COUNT + DOUBLE_TOUCH_MAX_GESTURES_COUNT];
static int cs_gesturesCount = 0;

static rect_area_t csArea;

/*cypress */
gesture_slot_t slots[MAX_SLOTS_COUNT];
static int currentSlot = 0;
static bool slot_changes = false;
static struct input_mt_slot slot_events[MAX_SLOTS_COUNT];
//static bool lastFilterResult = false;

static touch_accumulator_t inputTouches[MAX_SLOTS_RECOGNITION];

static rect_area_t topArea;
static rect_area_t bottomArea;

static touch_timer_t longTouchTimer;
//struct timer_list fs_lock_event_timer;

static global_status_t globalStatus;

struct input_dev *gestureInputDev = NULL;
struct input_dev *virtualTPInputDev = NULL;

//static gesture_input_event_t inputBuff[GESTURE_INPUT_EVENT_BUFF_SIZE];
//static int inputBuffCount = 0;

single_gesture_t singleTouchGestures[SINGLE_TOUCH_MAX_GESTURES_COUNT];
int singleTouchGesturesCount = 0;

double_touch_gesture_t doubleTouchGestures[DOUBLE_TOUCH_MAX_GESTURES_COUNT];
int doubleTouchGesturesCount = 0;

gesture_t gestures[SINGLE_TOUCH_MAX_GESTURES_COUNT + DOUBLE_TOUCH_MAX_GESTURES_COUNT];
int gesturesCount = 0;

touch_accumulator_t tempTouchAcc, tempTouchAcc2;

static key_info_t fsKeyInfo;
log_level_t log_level = LOG_LEVEL_ERROR;

screen_orientation_t orient_state;

/* enable/disable switch */
int gestFsEnabled = 1;

int filterEnabled = 0;
//int blockFsEvents = false;

extern bs_gesture_status_t bsStatus;

/*Capsense Functions */
static bool capsense_handle_touch(void);
static void cs_init_engine(void);
static void cs_load_gestures(void);
static long cs_compare_gestures(touch_accumulator_t *, single_gesture_t *, single_gesture_t *,
        compare_gesture_mode_t);
static all_gesture_compare_result_t cs_Compare_all_gestures(compare_gesture_mode_t);
static bool cs_is_inside_extended_areas(input_point_t *);
static void cs_add_new_point(int);
static void cs_update_gesture_type(void);
static void cs_update_available_gestures(void);
static int cs_get_valid_gestures_count(void);
static int cs_get_used_slots_count(void);
static bool capsense_gesture_driver_filter(unsigned int type, unsigned int code, int value);
static bool capsense_gesture_driver_event(unsigned int, unsigned int, int);


/* Cypress Functions */
static bool handle_touch(void);
static void init_engine(void);
static void fs_load_gestures(void);
static long get_gesture_length(touch_accumulator_t *);
//void copy_touch_accumulator(touch_accumulator_t *, touch_accumulator_t *);
//long match(single_gesture_t *, single_gesture_t *);
static long compare_gestures(touch_accumulator_t *, single_gesture_t *, single_gesture_t *,
        compare_gesture_mode_t);
static all_gesture_compare_result_t compare_all_gestures(compare_gesture_mode_t);
static long gesture_dot_product(single_gesture_t *, single_gesture_t *);
static long sqrt_l(long);

static bool is_inside_area(input_point_t *, rect_area_t *);
static bool is_inside_extended_areas(input_point_t *);
static int gesture_ts_init(void);
static void gesture_ts_exit(void);

static void add_new_point(int);
static void update_gesture_type(void);
static void long_touch_callback(unsigned long);
static void update_available_gestures(void);
static void disable_dt_gesture(void);
static int get_valid_gestures_count(void);
//static void fs_lock_event_callback(unsigned long data);


static long compare_dt_gestures(touch_accumulator_t *, touch_accumulator_t *, single_gesture_t *,
        single_gesture_t *, double_touch_gesture_t *, compare_gesture_mode_t);
static int get_used_slots_count(void);
static bool gesture_driver_filter(unsigned int type, unsigned int code, int value);
static bool fs_handle_key_main(void);
static bool fs_handle_recognized_gesture(void);
static bool front_gesture_driver_event(unsigned int, unsigned int, int);
static int virtual_input_device_init(void);
static void virtual_input_device_exit(void);
static bool gesture_driver_event(struct input_handle *handle, unsigned int type, unsigned int code,
                                 int value);
static bool gesture_driver_match(struct input_handler *handler, struct input_dev *dev);
static int gesture_driver_connect(struct input_handler *handler, struct input_dev *dev,
                                  const struct input_device_id *id);
static void gesture_driver_disconnect(struct input_handle *handle);


const struct input_device_id gesture_driver_ids[] = {
    { .driver_info = 1 },
    {},         /* Terminating zero entry */
};

struct input_handler gesture_driver_handler = {
    .filter = gesture_driver_event,
    .match = gesture_driver_match,
    .connect =  gesture_driver_connect,
    .disconnect =   gesture_driver_disconnect,
    .name =     "gesture_filter",
    .id_table = gesture_driver_ids,
};

/*
 * Gesture driver event handler callback
 */
static bool gesture_driver_event(struct input_handle *handle, unsigned int type, unsigned int code,
                                 int value) {

	bool result = false;
	if (strcmp(FS_INPUT_DRIVER_NAME,handle->dev->name) == 0){
	result = front_gesture_driver_event(type, code, value);
	}
	else if (strcmp(BS_INPUT_DRIVER_NAME,handle->dev->name) == 0)
	{
	result = back_gesture_driver_event(type,code,value);
	}
	else if (strcmp(CAPS_INPUT_DRIVER_NAME,handle->dev->name) == 0)
	{
	result = capsense_gesture_driver_event(type,code,value);
	}
	return result;
}

/*
 * Front gesture event handler function
 */
 static bool front_gesture_driver_event(unsigned int type, unsigned int code,
                                 int value) {

    bool filterResult;
    int valueOrient;

    LOG_V("cyttsp5_mt_front:type: %d, code: %d, value: %d\n",type, code, value);

    // Screen orientation changes
    valueOrient = value;
    if (orient_state == SCREEN_ORIENTATION_ROTATE_180) {
        if (type == EV_ABS && code == ABS_MT_POSITION_X) {
            valueOrient = TOP_ZONE_RIGHT_BOTTOM_X - value;
        } else if (type == EV_ABS && code == ABS_MT_POSITION_Y) {
            valueOrient = BOTTOM_ZONE_RIGHT_BOTTOM_Y - value;
        }
    }


    filterResult = gesture_driver_filter(type, code, valueOrient);
#if 0
    if (type == EV_SYN && code == SYN_REPORT) {
        LOG_V("Sync report, filterResult - %d, lastResult - %d\n", filterResult, lastFilterResult);
        if (filterResult) {
            if(lastFilterResult == false) {
                // we start filter, send touch up event
                int j, last_slot = -1;
                LOG_V("Start filtering!!!!\n");
                for (j = 0; j < MAX_SLOTS_COUNT; j++) {
                    if (slots[j].trackingId >= 0) {
                        last_slot = j;
                    }
                }
                if(last_slot == -1) {
                    LOG_V("Send touch up for recognized single finger gesture\n");
                    input_event(virtualTPInputDev, EV_ABS, ABS_MT_TRACKING_ID, -1);
                    input_event(virtualTPInputDev, EV_KEY, BTN_TOUCH, 0);
                    input_event(virtualTPInputDev, EV_SYN, SYN_REPORT, 0);
                } else {
                    for (j = 0; j < MAX_SLOTS_COUNT; j++) {
                        // if slot is active send touch up
                        if (slots[j].trackingId >= 0) {
                            LOG_V("Send touch up for slot - %d\n", j);
                            input_event(virtualTPInputDev, EV_ABS, ABS_MT_SLOT, j);
                            input_event(virtualTPInputDev, EV_ABS, ABS_MT_TRACKING_ID, -1);
                            if (j == last_slot) {
                                // send the BTN_TOUCH UP because original one will be filtered
                                input_event(virtualTPInputDev, EV_KEY, BTN_TOUCH, 0);
                            }

                            input_event(virtualTPInputDev, EV_SYN, SYN_REPORT, 0);
                        }
                    }
                    // reset current slot to zero
                    input_event(virtualTPInputDev, EV_ABS, ABS_MT_SLOT, 0);
                    input_event(virtualTPInputDev, EV_SYN, SYN_REPORT, 0);
                }
            }
            //clear buffer
			inputBuffCount = 0;
        } else {
            int i;
            // send all stored events and sync even
            if (!(bsStatus.fs_unlock_enabled && blockFsEvents)){
                if (virtualTPInputDev != NULL) {
                    for (i = 0; i < inputBuffCount; i++) {
                        input_event(virtualTPInputDev, inputBuff[i].type, inputBuff[i].code,
                              inputBuff[i].value);
                    }
                    input_event(virtualTPInputDev, type, code, value);
                    inputBuffCount = 0;
                 }
             }
        }
        lastFilterResult = filterResult;
    } else {
        if (inputBuffCount < GESTURE_INPUT_EVENT_BUFF_SIZE) {
            inputBuff[inputBuffCount].code = code;
            inputBuff[inputBuffCount].type = type;
            inputBuff[inputBuffCount].value = value;
            inputBuffCount++;
        } else {
            LOG_E("Error, input buffer overload\n");
        }
    }
#endif
        if (virtualTPInputDev != NULL) {
            input_event(virtualTPInputDev, type, code, value);
	 }

    // Filter out all events since we connected only to original touchscreen
    if(filterEnabled == 0) {
        return true;
    } else {
        return false;
    }
}

/*
 * capsense driver event handler function
 */
static bool capsense_gesture_driver_event(unsigned int type, unsigned int code,
                                 int value) {
	bool filterResult;

	LOG_V("cyttsp3-i2c:type=%d,code=%d,value=%d\n",type,code,value);

	filterResult = capsense_gesture_driver_filter(type, code, value);
	return true;
}

/*
 * Gesture driver event filter callback function
 */
bool gesture_driver_filter(unsigned int type, unsigned int code, int value) {

    struct timespec ts;
    struct timeval tv;
    ktime_get_ts(&ts);
    tv.tv_sec = ts.tv_sec;
    tv.tv_usec = ts.tv_nsec / NSEC_PER_USEC;

    if (type == EV_ABS) {
        if (code == ABS_MT_SLOT) {
            currentSlot = value;
            return false;
        }

        if (currentSlot < 0 || currentSlot >= MAX_SLOTS_COUNT) {
            LOG_E("error, slot is incorrect\n");
        } else {
            gesture_slot_t* sl = &slots[currentSlot];
            if (input_is_mt_value(code)) {
                slot_events[currentSlot].abs[code - ABS_MT_FIRST] = value;
            }
            sl->time = tv;
            switch (code) {
            case ABS_MT_POSITION_X:
                sl->changed = true;
                sl->positionX = value;
                break;
            case ABS_MT_POSITION_Y:
                sl->changed = true;
                sl->positionY = value;
                break;
            case ABS_MT_TRACKING_ID:
                slot_changes = true;
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
        // store key info
        fsKeyInfo.code = code;
        fsKeyInfo.value = value;
        fsKeyInfo.changed = true;

    }else if (type == EV_SYN && code == SYN_REPORT) {
        fs_handle_key_main();
        // Don't load CPU when we know this is not gesture
        if (!slot_changes) {
            switch(globalStatus.compareResult) {
			LOG_V("gesture_driver_filter : globalStatus.compareResult=%d\n",globalStatus.compareResult);
                case COMPARE_RESULT_NOT_GESTURE:return false;
                case COMPARE_RESULT_NOT_GESTURE_FILTERED:return true;
                default:break;
            }
        }
        slot_changes = false;
        return handle_touch();
    }
    return false;
}

/*
 * Capsense Gesture driver event filter callback function
 */
bool capsense_gesture_driver_filter(unsigned int type, unsigned int code, int value) {

    struct timespec ts;
    struct timeval tv;
    ktime_get_ts(&ts);
    tv.tv_sec = ts.tv_sec;
    tv.tv_usec = ts.tv_nsec / NSEC_PER_USEC;

	 if (type == EV_ABS) {
            gesture_slot_t* sl = &capsense_slots[CurrentCapsenseSlot];
            sl->time = tv;

            switch (code) {
            case ABS_MT_POSITION_X:
                sl->changed = true;
                sl->positionX = value;
                break;
            case ABS_MT_POSITION_Y:
                sl->changed = true;
                sl->positionY = value;
                break;
            case ABS_MT_TRACKING_ID:
                CS_slot_changes = true;
                if ((value < 0) && (capsenseSlots != DOUBLE_SLOT)) {
                    // The slot is no longer in use but it retains its previous contents,
                    // which may be reused for subsequent touches.
                    sl->status = SLOT_STATUS_TOUCH_UP;
                    sl->changed = true;
                } else {
                    sl->changed = true;
                    sl->trackingId = value;
		     if(value == DOUBLE_SLOT){
			capsenseSlots = value;
		       }
                }
                break;
            }
    }else if (type == EV_SYN && code == SYN_REPORT) {
        // Don't load CPU when we know this is not gesture
        if (!CS_slot_changes) {
            switch(cs_globalStatus.compareResult) {
			LOG_V("capsense_gesture_driver_filter : cs_globalStatus.compareResult=%d\n",cs_globalStatus.compareResult);
                case COMPARE_RESULT_NOT_GESTURE:return false;
                case COMPARE_RESULT_NOT_GESTURE_FILTERED:return true;
                default:break;
            }
        }
        CS_slot_changes = false;
        return capsense_handle_touch();
    }
    return false;
}

/*
 * Capsense touch handle function, recognize gesture,
 * return filter result
 */
bool capsense_handle_touch() {
    gesture_slot_t* slot;
    bool result = false;
    int i;
    bool newEvent = false;

    cs_update_gesture_type();
    cs_update_available_gestures();

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        slot = &capsense_slots[i];
        if (slot->status == SLOT_STATUS_NOT_USED && slot->changed) {
			// new touch
            LOG_D("new touch in  slot %d  X=%d, y=%d, tracking ID = %d\n", i, slot->positionX,
                    slot->positionY, slot->trackingId);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                cs_add_new_point(i);
            }
            slot->changed = 0;
            slot->status = SLOT_STATUS_NEW_TOUCH;
        } else if ((slot->status == SLOT_STATUS_NEW_TOUCH || slot->status == SLOT_STATUS_CHANGED)
                && slot->changed) {
			//change
            LOG_D("change in current slot %d  X=%d, y=%d, tracking ID = %d\n", i, slot->positionX,
                    slot->positionY, slot->trackingId);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                cs_add_new_point(i);
            }
            slot->status = SLOT_STATUS_CHANGED;
            slot->changed = 0;
        } else if (slot->status == SLOT_STATUS_TOUCH_UP && slot->changed) {
			// this is touch up
            LOG_D("touch up detected for slot - %d \n", i);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                cs_add_new_point(i);
            }
            slot->status = SLOT_STATUS_NOT_USED;
            slot->trackingId = -1;
            slot->changed = false;
        }

    }

    if (!newEvent) {
        // return if don't have new events
        LOG_V("No new events, cs_globalStatus.compareResult - %d\n", cs_globalStatus.compareResult);
        switch (cs_globalStatus.compareResult) {
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
    cs_update_gesture_type();

    switch (cs_globalStatus.gestureType) {
    case GESTURE_TYPE_SINGLE_OR_DOUBLE:
    case GESTURE_TYPE_SINGLE:
    case GESTURE_TYPE_DOUBLE:
        switch (cs_globalStatus.compareResult) {
        case COMPARE_RESULT_GESTURE:
            // notify and filter it
		    result = true;
            if (cs_globalStatus.action < 0) {
                // already sent
                break;
            }
            if (cs_globalStatus.needSendKeyDown) {
                cs_globalStatus.delayedKeyUP = cs_globalStatus.action;
                LOG_V("send single key down\n");
                report_key_down(cs_globalStatus.action);
                cs_globalStatus.action = -1;
            }
            break;
        case COMPARE_RESULT_NOT_GESTURE:
        case COMPARE_RESULT_NOT_ACTIVE:
        case COMPARE_RESULT_NOT_RECOGNIZED:
            result = false;
            break;
        case COMPARE_RESULT_NOT_GESTURE_FILTERED:
            result = true;
            break;
        case COMPARE_RESULT_PART_OF_GESTURE:
            // probably gesture
            // moved to screen area, disable double touch gestures
            if (cs_get_valid_gestures_count() == 0) {
                cs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE_FILTERED;
            }
            result = true;
            break;
        }
        break;
    case GESTURE_TYPE_NOT_GESTURE:
        switch (cs_globalStatus.compareResult) {
        case COMPARE_RESULT_PART_OF_GESTURE:
        case COMPARE_RESULT_GESTURE:
        case COMPARE_RESULT_NOT_GESTURE_FILTERED:
            result = true;
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
    if (cs_globalStatus.compareResult == COMPARE_RESULT_GESTURE
            && (cs_globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
                    || cs_globalStatus.gestureType == GESTURE_TYPE_SINGLE)
            && capsense_slots[0].status == SLOT_STATUS_NOT_USED && capsenseSlots != DOUBLE_TOUCH_SLOT) {
        if (cs_globalStatus.needSendKeyDown && cs_globalStatus.delayedKeyUP >= 0) {
			LOG_V("send  key up\n");
            report_key_up(cs_globalStatus.delayedKeyUP);
            cs_globalStatus.delayedKeyUP = -1;
        } else {
            report_key(cs_globalStatus.action);
        }

    }

// make all gestures available if there are no touches.
    if (cs_get_used_slots_count() == 0) {
        for (i = 0; i < cs_gesturesCount; i++) {
            cs_Gestures[i].status = GESTURE_STATUS_VALID;

        }
        cs_globalStatus.gestureType = GESTURE_TYPE_SINGLE_OR_DOUBLE;
        cs_globalStatus.compareResult = COMPARE_RESULT_NOT_ACTIVE;
        cs_globalStatus.action = -1;
        cs_globalStatus.delayedKeyUP = -1;
    }

    LOG_V("capsense handletouch filter result - %d\n", result);
    LOG_V("compare_result = %d\n", cs_globalStatus.compareResult);
    LOG_V("gesture type = %d\n", cs_globalStatus.gestureType);
    return result;
}

/*
 * Update type of gesture that we will recognize
 */
void cs_update_gesture_type(void) {
    int i;
    gesture_slot_t * sl;
    int slotsCount = 0;

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        sl = &capsense_slots[i];
        if (sl->status != SLOT_STATUS_NOT_USED) {
		slotsCount++;
        } else if (sl->changed) {
            slotsCount++;
        }
    }

    switch (cs_globalStatus.compareResult) {
    case COMPARE_RESULT_NOT_GESTURE:
	break;
    case COMPARE_RESULT_GESTURE:
        //don't change gesture type if we already recognized gesture
        break;
    case COMPARE_RESULT_NOT_ACTIVE:
    case COMPARE_RESULT_NOT_RECOGNIZED:
        if (slotsCount == MAX_SLOTS_RECOGNITION) {
            cs_globalStatus.gestureType = GESTURE_TYPE_DOUBLE;
        } else if (slotsCount > MAX_SLOTS_RECOGNITION) {
            cs_globalStatus.gestureType = GESTURE_TYPE_NOT_GESTURE;
        }
        break;
    case COMPARE_RESULT_PART_OF_GESTURE:
        // we should have decision about gesture type here
        if ((cs_globalStatus.gestureType == GESTURE_TYPE_DOUBLE
                && slotsCount != MAX_SLOTS_RECOGNITION)
                || (cs_globalStatus.gestureType == GESTURE_TYPE_SINGLE && slotsCount != 1)) {
            cs_globalStatus.gestureType = GESTURE_TYPE_NOT_GESTURE;
        }
        break;

    case COMPARE_RESULT_NOT_GESTURE_FILTERED:
        break;
    }
}


/*
 * Update available gesture list based on current type
 */
void cs_update_available_gestures(void) {

    int i;
    bool cancelST = false;
    bool cancelDT = false;

    switch (cs_globalStatus.gestureType) {
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

    for (i = 0; i < cs_gesturesCount; i++) {
        if (cancelST && cs_Gestures[i].singleGesture != 0
                && cs_Gestures[i].status != GESTURE_STATUS_RECOGNIZED) {
            cs_Gestures[i].status = GESTURE_STATUS_NOT_VALID;
        }
    }
}


/*
 * Add Capsense new point, call gesture recognition if necessary
 * Update gesture compare_result.
 */
void cs_add_new_point(int inputId) {

    gesture_slot_t * slot;
    slot = &capsense_slots[inputId];
    switch (cs_globalStatus.compareResult) {
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
            if (cs_globalStatus.compareResult == COMPARE_RESULT_PART_OF_GESTURE) {
                // cann't accept new touch in this state
                cs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE_FILTERED;
                break;
            }
	     LOG_V("add first point\n");
            cs_inputTouches[inputId].pointsCount = 1;
            cs_inputTouches[inputId].points[0].x = slot->positionX;
            cs_inputTouches[inputId].points[0].y = slot->positionY;
            cs_inputTouches[inputId].startPoint.x = slot->positionX;
            cs_inputTouches[inputId].startPoint.y = slot->positionY;
            cs_inputTouches[inputId].endPoint = cs_inputTouches[inputId].startPoint;
            cs_inputTouches[inputId].minX = slot->positionX;
            cs_inputTouches[inputId].minY = slot->positionY;
            cs_inputTouches[inputId].maxX = cs_inputTouches[inputId].minX;
            cs_inputTouches[inputId].maxY = cs_inputTouches[inputId].minY;
            cs_inputTouches[inputId].startTime = slot->time;
            cs_inputTouches[inputId].endTime = slot->time;

            if (!cs_is_inside_extended_areas(&cs_inputTouches[inputId].startPoint)) {
                cs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
            } else {
                cs_globalStatus.compareResult = COMPARE_RESULT_NOT_RECOGNIZED;
            }
            break;
        case SLOT_STATUS_NEW_TOUCH:
        case SLOT_STATUS_CHANGED:
        case SLOT_STATUS_TOUCH_UP:
            // add new point
            if (cs_inputTouches[inputId].pointsCount >= TOUCH_ACC_MAX_POINTS_COUNT) {
                // too many points, break
                //TODO something
                LOG_E("Too many points...break\n");
                break;
            }
            cs_inputTouches[inputId].points[cs_inputTouches[inputId].pointsCount].x = slot->positionX;
            cs_inputTouches[inputId].points[cs_inputTouches[inputId].pointsCount].y = slot->positionY;
            cs_inputTouches[inputId].pointsCount++;
            cs_inputTouches[inputId].endTime = slot->time;
            cs_inputTouches[inputId].endPoint.x = slot->positionX;
            cs_inputTouches[inputId].endPoint.y = slot->positionY;

            // update min and max values
            if (slot->positionX < cs_inputTouches[inputId].minX) {
                cs_inputTouches[inputId].minX = slot->positionX;
            } else if (slot->positionX > cs_inputTouches[inputId].maxX) {
                cs_inputTouches[inputId].maxX = slot->positionX;
            }

            if (slot->positionY < cs_inputTouches[inputId].minY) {
                cs_inputTouches[inputId].minY = slot->positionY;
            } else if (slot->positionY > cs_inputTouches[inputId].maxY) {
                cs_inputTouches[inputId].maxY = slot->positionY;
            }

	        if (slot->status == SLOT_STATUS_TOUCH_UP) {
                all_gesture_compare_result_t result;
                result = cs_Compare_all_gestures(COMPARE_GESTURE_ALL);
                if (result.keyId >= 0) {
                    cs_globalStatus.action = result.keyId;
                    cs_globalStatus.needSendKeyDown = result.needSendKeyDown;
                    cs_globalStatus.compareResult = COMPARE_RESULT_GESTURE;
                }

            }
            // check last point area
            if (cs_is_inside_extended_areas(
                    &cs_inputTouches[inputId].points[cs_inputTouches[inputId].pointsCount - 1])) {
                // we are in extended area now
                if (!cs_is_inside_extended_areas(&cs_inputTouches[inputId].startPoint)) {
                    // started from screen area and moved to extended area, not a gesture
                    cs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                }
            } else {
                if (cs_is_inside_extended_areas(&cs_inputTouches[inputId].startPoint)) {
                    // started from extended areas and moved to screen area.
                    // need to check for gesture
                    LOG_V("started in ext areas and moved to screen area...need to check for gesture\n");
                    if (cs_globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE) {
                        LOG_V("gesturetype is singleorDoube...update it to single and disable double touch gesture\n");
						cs_globalStatus.gestureType = GESTURE_TYPE_SINGLE;
                        //bs_disable_dt_gesture();
                    }
                    LOG_V("Moved to screen , gesture type- %d\n", cs_globalStatus.gestureType);

                    if (cs_globalStatus.compareResult == COMPARE_RESULT_PART_OF_GESTURE) {
                        all_gesture_compare_result_t result;
                        result = cs_Compare_all_gestures(COMPARE_GESTURE_ALL_MOVING);
                        if (result.keyId >= 0) {
                            cs_globalStatus.action = result.keyId;
                            cs_globalStatus.needSendKeyDown = result.needSendKeyDown;
                            cs_globalStatus.compareResult = COMPARE_RESULT_GESTURE;
                        }
                    } else if (cs_globalStatus.compareResult == COMPARE_RESULT_NOT_RECOGNIZED) {
                        all_gesture_compare_result_t result;
                        result = cs_Compare_all_gestures(COMPARE_GESTURE_AND_START_POINT);
                        if (result.keyId >= 0) {
                            cs_globalStatus.action = result.keyId;
                            cs_globalStatus.needSendKeyDown = result.needSendKeyDown;
                            cs_globalStatus.compareResult = COMPARE_RESULT_PART_OF_GESTURE;
			} else {
                            cs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                        }
                    }
                } else {
                    // started from screen area, it is not a gesture
                    cs_globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                }
            }
            break;
        }
        break;
    }
	LOG_V("add new point result input0 - %d\n", cs_globalStatus.compareResult);
}

/*
 * Check if point is inside capsesne extended area.
 */
bool cs_is_inside_extended_areas(input_point_t * point) {
    return (is_inside_area(point, &csArea));
}

/*
 * Compare user input with all valid capsense gestures
 */
all_gesture_compare_result_t cs_Compare_all_gestures(compare_gesture_mode_t mode) {

    single_gesture_t tempGesture, tempGesture2;
    all_gesture_compare_result_t cResult;
    bool separateKeyUPKeyDown = false;
    long result;
    int i;
    int maxGestureValue = 0;
    int maxGestureKeyId = -1;

    copy_touch_accumulator(&cs_inputTouches[0], &cs_tempTouchAcc);
    normalize_size(&cs_tempTouchAcc);
    normalize_spacing(&tempGesture, &cs_tempTouchAcc);
    normalize_center(&tempGesture);

    if (cs_globalStatus.gestureType == GESTURE_TYPE_DOUBLE) {
        copy_touch_accumulator(&cs_inputTouches[1], &cs_tempTouchAcc2);
        normalize_size(&cs_tempTouchAcc2);
        normalize_spacing(&tempGesture2, &cs_tempTouchAcc2);
        normalize_center(&tempGesture2);
    }

    for (i = 0; i < cs_gesturesCount; i++) {
        if (cs_Gestures[i].status == GESTURE_STATUS_VALID) {
            if (cs_Gestures[i].singleGesture != 0
                    && (cs_globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
                            || cs_globalStatus.gestureType == GESTURE_TYPE_SINGLE)) {
                result = cs_compare_gestures(&cs_tempTouchAcc, &tempGesture, cs_Gestures[i].singleGesture,
                        mode);
                if (result == GESTURE_COMPARE_NOT_GESTURE) {
                    cs_Gestures[i].status = GESTURE_STATUS_NOT_VALID;
                    continue;
                }
                if (result > maxGestureValue) {
                    maxGestureValue = result;
                    maxGestureKeyId = cs_Gestures[i].singleGesture->keyId;
                    separateKeyUPKeyDown = cs_Gestures[i].singleGesture->needSendKeyDown;
                }
            }

        }
    }
    LOG_V("Compare all gesture: max result - %d, key ID - %d\n", maxGestureValue, maxGestureKeyId);
    if (maxGestureValue > GESTURE_RECOGNITION_THRESHOLD) {
        // gesture is recognized
	cResult.keyId = maxGestureKeyId;
        cResult.needSendKeyDown = separateKeyUPKeyDown;
    } else {
        cResult.keyId = -1;
        cResult.needSendKeyDown = false;
    }
    return cResult;
}

/*
 * Compare user input with capsense gesture
 */
long cs_compare_gestures(touch_accumulator_t * gesture1, single_gesture_t * inputGesture,
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

    if (!is_inside_area(&gesture1->startPoint, &gesture2->startPointArea)) {
        LOG_V("Start point out of gesture start area, x- %lu,  y- %lu\n", gesture1->startPoint.x,
                gesture1->startPoint.y);
		LOG_V("Recognize gesture's start point is not inside area of defined gestures,Return GESTURE_COMPARE_NOT_GESTURE !!\n");
        return GESTURE_COMPARE_NOT_GESTURE;
    }

    if (mode == COMPARE_GESTURE_ALL || mode == COMPARE_GESTURE_LONG_TOUCH
            || mode == COMPARE_GESTURE_ALL_MOVING) {
		LOG_V("mode=%d\n",mode);
        if (gesture2->endPointArea.leftUpX != -1) {
            if (!is_inside_area(&gesture1->endPoint, &gesture2->endPointArea)) {
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

/*
 * Return count of all valid gestures
 */
int cs_get_valid_gestures_count(void) {
    int i, count;
    count = 0;
    for (i = 0; i < cs_gesturesCount; i++) {
        if (cs_Gestures[i].status != GESTURE_STATUS_NOT_VALID) {
            count++;
        }
    }
    return count;
}


/*
 * Get currently used slots count
 */
int cs_get_used_slots_count(void) {
    int i;
    int slotsCount = 0;

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        if (capsense_slots[i].status != SLOT_STATUS_NOT_USED) {
            slotsCount++;
        } else if (capsense_slots[i].changed) {
            slotsCount++;
        }
    }
    return slotsCount;
}

/*
 * Main key handler
 */
bool fs_handle_key_main(void) {
    if (fsKeyInfo.changed == false) {
        LOG_V("key has not been changed\n");
        return false;
    }
    fsKeyInfo.changed = false;

    if (fs_handle_recognized_gesture() == true) {
        LOG_V("it is mcu recognized gesture\n");
        return true;
    }

/*#ifdef BS_FORWARD_RAW_KEY_EVENTS
	LOG_V("input_report_key for keyInfo.code=%d,keyInfo.value=%d\n",keyInfo.code,keyInfo.value);
    input_report_key(gestureInputDev, keyInfo.code, keyInfo.value);
    input_sync(gestureInputDev);
#endif
*/
//input_event(virtualTPInputDev, EV_KEY, fsKeyInfo.code, fsKeyInfo.value); Jitendra

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
bool fs_handle_recognized_gesture(void) {
    if (fsKeyInfo.code >= KEY_BS_GESTURE_MINIMUM && fsKeyInfo.code <= KEY_BS_GESTURE_MAXIMUM) {
        // it is recognized gesture key event, resend it to android
	input_report_key(gestureInputDev, fsKeyInfo.code, fsKeyInfo.value);
        input_sync(gestureInputDev);
        return true;
    }
    return false;
}

/*
 * Gesture driver connect callback function.
 */
static int gesture_driver_connect(struct input_handler *handler, struct input_dev *dev,
        const struct input_device_id *id) {
    struct input_handle *handle;
    int error;
    handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
    if (!handle)
        return -ENOMEM;

    handle->dev = dev;
    handle->handler = handler;
    handle->name = "Gesture driver";

    error = input_register_handle(handle);
    if (error)
        goto err_free_handle;

    error = input_open_device(handle);
    if (error)
        goto err_unregister_handle;

    LOG_V("Connected device: %s (%s at %s)\n",
            dev_name(&dev->dev),
            dev->name ?: "unknown",
            dev->phys ?: "unknown");

    return 0;

    err_unregister_handle: input_unregister_handle(handle);
    err_free_handle: kfree(handle);
    return error;
}

/*
 * Gesture driver disconnect callback function
 */
static void gesture_driver_disconnect(struct input_handle *handle) {
    LOG_V("Disconnected device: %s\n", dev_name(&handle->dev->dev));

    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
}

/*
 * Gesture driver match callback function
 */
static bool gesture_driver_match(struct input_handler *handler, struct input_dev *dev)
{
    //if ((strcmp(FS_INPUT_DRIVER_NAME,dev->name) == 0) ||(strcmp(BS_INPUT_DRIVER_NAME,dev->name) == 0)) {
    if ((strcmp(FS_INPUT_DRIVER_NAME,dev->name) == 0) || (strcmp(BS_INPUT_DRIVER_NAME,dev->name) == 0) || (strcmp(CAPS_INPUT_DRIVER_NAME,dev->name) == 0)) {
	return true;
    }
    return false;
}

/*
 * Gesture driver initialization
 */
static int __init gesture_driver_init(void)
{
    fs_load_gestures();
    gesture_ts_init();
    init_engine();
    cs_load_gestures();
    cs_init_engine();
    bs_load_gestures();
    bs_init_driver();
    virtual_input_device_init();
    bs_virtual_input_device_init();
    gesture_config_init();
    return input_register_handler(&gesture_driver_handler);
}

/*
 * Gesture driver exit function
 */
static void __exit gesture_driver_exit(void)
{
    gesture_config_terminate();
    del_timer( &longTouchTimer.timer );
    gesture_ts_exit();
    virtual_input_device_exit();
    bs_virtual_input_device_exit();
    input_unregister_handler(&gesture_driver_handler);
}

/*
 * Gesture recognition engine initialization
 */
void init_engine(void) {
    gesture_slot_t* slot;
    int i;

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        slot = &slots[i];
        slot->status = SLOT_STATUS_NOT_USED;
        slot->trackingId = -1;
        slot->changed = false;
    }

    topArea.leftUpX = TOP_ZONE_LEFT_TOP_X;
    topArea.leftUpY = TOP_ZONE_LEFT_TOP_Y;
    topArea.rightBottomX = TOP_ZONE_RIGHT_BOTTOM_X;
    topArea.rightBottomY = TOP_ZONE_RIGHT_BOTTOM_Y;

    bottomArea.leftUpX = BOTTOM_ZONE_LEFT_TOP_X;
    bottomArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y;
    bottomArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    bottomArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;

    for (i = 0; i < MAX_SLOTS_RECOGNITION; i++) {
        inputTouches[i].pointsCount = 0;
    }

    //setup long touch timer
    setup_timer(&longTouchTimer.timer, long_touch_callback, 0);

    //setup_timer(&fs_lock_event_timer, fs_lock_event_callback, 0);

    globalStatus.gestureType = GESTURE_TYPE_SINGLE_OR_DOUBLE;
    globalStatus.compareResult = COMPARE_RESULT_NOT_ACTIVE;

    orient_state = SCREEN_ORIENTATION_ROTATE_0;
}
/*
 * Load predefined gestures
 */
void fs_load_gestures(void) {

    int i;

    // back swipe
    tempTouchAcc.pointsCount = 2;
    tempTouchAcc.points[0].x = 100;
    tempTouchAcc.points[0].y = 0;
    tempTouchAcc.points[1].x = 0;
    tempTouchAcc.points[1].y = 0;

    singleTouchGestures[0].touchAccPointsCount = tempTouchAcc.pointsCount;
    memcpy(singleTouchGestures[0].touchAccPoints, tempTouchAcc.points,\
        tempTouchAcc.pointsCount * sizeof(input_point_t));
    normalize_size(&tempTouchAcc);
    normalize_spacing(&singleTouchGestures[0], &tempTouchAcc);
    normalize_center(&singleTouchGestures[0]);
    // gesture constraints
    singleTouchGestures[0].minDx = 72;//10%
    singleTouchGestures[0].minDy = 0;
    singleTouchGestures[0].maxDx = 0;
    singleTouchGestures[0].maxDy = 0;
    singleTouchGestures[0].startPointArea.leftUpX = 0;
    singleTouchGestures[0].startPointArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y;
    singleTouchGestures[0].startPointArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    singleTouchGestures[0].startPointArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;
    singleTouchGestures[0].minTime = PAN_MIN_TIME;
    singleTouchGestures[0].maxTime = PAN_MAX_TIME;
    singleTouchGestures[0].endPointArea.leftUpX = 0;
    singleTouchGestures[0].endPointArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y-100;
    singleTouchGestures[0].endPointArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    singleTouchGestures[0].endPointArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;
    singleTouchGestures[0].keyId = ACTION_BACK_FLICK;
    singleTouchGestures[0].needSendKeyDown = false;
    singleTouchGestures[0].isLongTapGesture = false;
    singleTouchGestures[0].isDetectOnTouchUp = true;


    // home swipe
    tempTouchAcc.pointsCount = 2;
    tempTouchAcc.points[0].x = 0;
    tempTouchAcc.points[0].y = 0;
    tempTouchAcc.points[1].x = 100;
    tempTouchAcc.points[1].y = 0;

    singleTouchGestures[1].touchAccPointsCount = tempTouchAcc.pointsCount;
    memcpy(singleTouchGestures[1].touchAccPoints, tempTouchAcc.points,\
        tempTouchAcc.pointsCount * sizeof(input_point_t));
    normalize_size(&tempTouchAcc);
    normalize_spacing(&singleTouchGestures[1], &tempTouchAcc);
    normalize_center(&singleTouchGestures[1]);
    // gesture constraints
    singleTouchGestures[1].minDx = 72;//10%
    singleTouchGestures[1].minDy = 0;
    singleTouchGestures[1].maxDx = 0;
    singleTouchGestures[1].maxDy = 0;
    singleTouchGestures[1].startPointArea.leftUpX = 0;
    singleTouchGestures[1].startPointArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y;
    singleTouchGestures[1].startPointArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    singleTouchGestures[1].startPointArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;
    singleTouchGestures[1].minTime = PAN_MIN_TIME;
    singleTouchGestures[1].maxTime = PAN_MAX_TIME;
    singleTouchGestures[1].endPointArea.leftUpX = 0;
    singleTouchGestures[1].endPointArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y - 100;
    singleTouchGestures[1].endPointArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    singleTouchGestures[1].endPointArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;
    singleTouchGestures[1].keyId = ACTION_HOME_PAN;
    singleTouchGestures[1].needSendKeyDown = false;
    singleTouchGestures[1].isLongTapGesture = false;
    singleTouchGestures[1].isDetectOnTouchUp = true;


    // recent apps, long touch
    // gesture constraints
    singleTouchGestures[2].minDx = 0;
    singleTouchGestures[2].minDy = 0;
    singleTouchGestures[2].maxDx = 50;
    singleTouchGestures[2].maxDy = 50;
    singleTouchGestures[2].startPointArea.leftUpX = BOTTOM_ZONE_LEFT_TOP_X;
    singleTouchGestures[2].startPointArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y;
    singleTouchGestures[2].startPointArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    singleTouchGestures[2].startPointArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;
    singleTouchGestures[2].minTime = 450;
    singleTouchGestures[2].maxTime = 550;
    singleTouchGestures[2].endPointArea.leftUpX = -1;
    singleTouchGestures[2].endPointArea.leftUpY = -1;
    singleTouchGestures[2].endPointArea.rightBottomX = -1;
    singleTouchGestures[2].endPointArea.rightBottomY = -1;
    singleTouchGestures[2].keyId = ACTION_RECENT_APPS_LONG_TOUCH;
    singleTouchGestures[2].needSendKeyDown = true;
    singleTouchGestures[2].isLongTapGesture = true;
    singleTouchGestures[2].isDetectOnTouchUp = false;


    // top area long touch - DISABLED
    // gesture constraints
    /*
    singleTouchGestures[4].minDx = 0;
    singleTouchGestures[4].minDy = 0;
    singleTouchGestures[4].maxDx = 50;
    singleTouchGestures[4].maxDy = 50;
    singleTouchGestures[4].startPointArea.leftUpX = TOP_ZONE_LEFT_TOP_X;
    singleTouchGestures[4].startPointArea.leftUpY = TOP_ZONE_LEFT_TOP_Y;
    singleTouchGestures[4].startPointArea.rightBottomX = TOP_ZONE_RIGHT_BOTTOM_X;
    singleTouchGestures[4].startPointArea.rightBottomY = TOP_ZONE_RIGHT_BOTTOM_Y;
    singleTouchGestures[4].minTime = 450;
    singleTouchGestures[4].maxTime = 550;
    singleTouchGestures[4].endPointArea.leftUpX = -1;
    singleTouchGestures[4].endPointArea.leftUpY = -1;
    singleTouchGestures[4].endPointArea.rightBottomX = -1;
    singleTouchGestures[4].endPointArea.rightBottomY = -1;
    singleTouchGestures[4].keyId = ACTION_TOP_AREA_LONG_TOUCH;
    singleTouchGestures[4].needSendKeyDown = true;
    singleTouchGestures[4].isTapGesture = true;
    singleTouchGestures[4].isDetectOnTouchUp = false;
     */

    // single tap , need for double tap
    singleTouchGestures[3].minDx = 0;
    singleTouchGestures[3].minDy = 0;
    singleTouchGestures[3].maxDx = 50;
    singleTouchGestures[3].maxDy = 50;
    singleTouchGestures[3].startPointArea.leftUpX = BOTTOM_ZONE_LEFT_TOP_X;
    singleTouchGestures[3].startPointArea.leftUpY = BOTTOM_ZONE_LEFT_TOP_Y;
    singleTouchGestures[3].startPointArea.rightBottomX = BOTTOM_ZONE_RIGHT_BOTTOM_X;
    singleTouchGestures[3].startPointArea.rightBottomY = BOTTOM_ZONE_RIGHT_BOTTOM_Y;
    singleTouchGestures[3].minTime = 50;
    singleTouchGestures[3].maxTime = 400;
    singleTouchGestures[3].endPointArea.leftUpX = -1;
    singleTouchGestures[3].endPointArea.leftUpY = -1;
    singleTouchGestures[3].endPointArea.rightBottomX = -1;
    singleTouchGestures[3].endPointArea.rightBottomY = -1;
    singleTouchGestures[3].keyId = KEY_FS_GESTURE_SINGLE_TAP;
    singleTouchGestures[3].needSendKeyDown = false;
    singleTouchGestures[3].isLongTapGesture = false;
    singleTouchGestures[3].isDetectOnTouchUp = true;


    singleTouchGesturesCount = 4;

    //*************************************************************
    // double touch gestures

    //put to back gesture
    tempTouchAcc.pointsCount = 2;
    tempTouchAcc.points[0].x = 0;
    tempTouchAcc.points[0].y = 0;
    tempTouchAcc.points[1].x = 0;
    tempTouchAcc.points[1].y = 1000;

    doubleTouchGestures[0].gesture1.touchAccPointsCount = tempTouchAcc.pointsCount;
    memcpy(doubleTouchGestures[0].gesture1.touchAccPoints, tempTouchAcc.points,\
        tempTouchAcc.pointsCount * sizeof(input_point_t));
    normalize_size(&tempTouchAcc);
    normalize_spacing(&doubleTouchGestures[0].gesture1, &tempTouchAcc);
    normalize_center(&doubleTouchGestures[0].gesture1);
    // gesture constraints
    doubleTouchGestures[0].gesture1.minDy = 600; // for demo build , should be TOP_ZONE_RIGHT_BOTTOM_Y/2
    doubleTouchGestures[0].gesture1.minDx = 0;
    doubleTouchGestures[0].gesture1.maxDy = 0;
    doubleTouchGestures[0].gesture1.maxDx = 0;
    doubleTouchGestures[0].gesture1.startPointArea.leftUpX = TOP_ZONE_LEFT_TOP_X;
    doubleTouchGestures[0].gesture1.startPointArea.leftUpY = TOP_ZONE_LEFT_TOP_Y;
    doubleTouchGestures[0].gesture1.startPointArea.rightBottomX = TOP_ZONE_RIGHT_BOTTOM_X;
    doubleTouchGestures[0].gesture1.startPointArea.rightBottomY = TOP_ZONE_RIGHT_BOTTOM_Y;
    doubleTouchGestures[0].gesture1.minTime = 200;
    doubleTouchGestures[0].gesture1.maxTime = PAN_MAX_TIME;
    doubleTouchGestures[0].gesture1.endPointArea.leftUpX = -1;
    doubleTouchGestures[0].gesture1.endPointArea.leftUpY = -1;
    doubleTouchGestures[0].gesture1.endPointArea.rightBottomX = -1;
    doubleTouchGestures[0].gesture1.endPointArea.rightBottomY = -1;
    doubleTouchGestures[0].gesture1.keyId = ACTION_PUT_TO_BACK;
    doubleTouchGestures[0].gesture1.needSendKeyDown = true;
    doubleTouchGestures[0].gesture1.isDetectOnTouchUp = false;

    tempTouchAcc.pointsCount = 2;
    tempTouchAcc.points[0].x = 0;
    tempTouchAcc.points[0].y = 0;
    tempTouchAcc.points[1].x = 0;
    tempTouchAcc.points[1].y = 1000;

    doubleTouchGestures[0].gesture2.touchAccPointsCount = tempTouchAcc.pointsCount;
    memcpy(doubleTouchGestures[0].gesture2.touchAccPoints, tempTouchAcc.points,\
        tempTouchAcc.pointsCount * sizeof(input_point_t));
    normalize_size(&tempTouchAcc);
    normalize_spacing(&doubleTouchGestures[0].gesture2, &tempTouchAcc);
    normalize_center(&doubleTouchGestures[0].gesture2);
    // gesture constraints
    doubleTouchGestures[0].gesture2.minDy = 600; // for demo build , should be TOP_ZONE_RIGHT_BOTTOM_Y/2
    doubleTouchGestures[0].gesture2.minDx = 0;
    doubleTouchGestures[0].gesture2.maxDy = 0;
    doubleTouchGestures[0].gesture2.maxDx = 0;
    doubleTouchGestures[0].gesture2.startPointArea.leftUpX = TOP_ZONE_LEFT_TOP_X;
    doubleTouchGestures[0].gesture2.startPointArea.leftUpY = TOP_ZONE_LEFT_TOP_Y;
    doubleTouchGestures[0].gesture2.startPointArea.rightBottomX = TOP_ZONE_RIGHT_BOTTOM_X;
    doubleTouchGestures[0].gesture2.startPointArea.rightBottomY = TOP_ZONE_RIGHT_BOTTOM_Y;
    doubleTouchGestures[0].gesture2.minTime = 200;
    doubleTouchGestures[0].gesture2.maxTime = PAN_MAX_TIME;
    doubleTouchGestures[0].gesture2.endPointArea.leftUpX = -1;
    doubleTouchGestures[0].gesture2.endPointArea.leftUpY = -1;
    doubleTouchGestures[0].gesture2.endPointArea.rightBottomX = -1;
    doubleTouchGestures[0].gesture2.endPointArea.rightBottomY = -1;
    doubleTouchGestures[0].gesture2.keyId = ACTION_PUT_TO_BACK;
    doubleTouchGestures[0].gesture2.needSendKeyDown = true;
    doubleTouchGestures[0].gesture2.isDetectOnTouchUp = false;

    doubleTouchGestures[0].keyId = ACTION_PUT_TO_BACK;
    doubleTouchGesturesCount = 1;
    //*************************************************************

    gesturesCount = 0;
    for (i = 0; i < singleTouchGesturesCount; i++,gesturesCount++) {
        gestures[gesturesCount].status = GESTURE_STATUS_VALID;
        gestures[gesturesCount].singleGesture = &singleTouchGestures[i];
        gestures[gesturesCount].dtGesture = 0;
    }

    for (i = 0; i < doubleTouchGesturesCount; i++,gesturesCount++) {
        gestures[gesturesCount].status = GESTURE_STATUS_VALID;
        gestures[gesturesCount].singleGesture = 0;
        gestures[gesturesCount].dtGesture = &doubleTouchGestures[i];
    }

}

/*
 * Load capsense gestures
 */
void cs_load_gestures(void) {

    int i;

    // Right to left swipe
    cs_tempTouchAcc.pointsCount = 2;
    cs_tempTouchAcc.points[0].x = 100;
    cs_tempTouchAcc.points[0].y = 0;
    cs_tempTouchAcc.points[1].x = 0;
    cs_tempTouchAcc.points[1].y = 0;

    cs_singleTouchGestures[0].touchAccPointsCount = cs_tempTouchAcc.pointsCount;
    memcpy(cs_singleTouchGestures[0].touchAccPoints, cs_tempTouchAcc.points,\
        cs_tempTouchAcc.pointsCount * sizeof(input_point_t));

    normalize_size(&cs_tempTouchAcc);
    normalize_spacing(&cs_singleTouchGestures[0], &cs_tempTouchAcc);
    normalize_center(&cs_singleTouchGestures[0]);
    // gesture constraints
    cs_singleTouchGestures[0].minDx = 72;//10%
    cs_singleTouchGestures[0].minDy = 0;
    cs_singleTouchGestures[0].maxDx = 0;
    cs_singleTouchGestures[0].maxDy = 0;
    cs_singleTouchGestures[0].startPointArea.leftUpX = 0;
    cs_singleTouchGestures[0].startPointArea.leftUpY = CS_ZONE_LEFT_TOP_Y;
    cs_singleTouchGestures[0].startPointArea.rightBottomX = CS_ZONE_RIGHT_BOTTOM_X;
    cs_singleTouchGestures[0].startPointArea.rightBottomY = CS_ZONE_RIGHT_BOTTOM_Y;
    cs_singleTouchGestures[0].minTime = 50;
    cs_singleTouchGestures[0].maxTime = 3000;
    cs_singleTouchGestures[0].endPointArea.leftUpX = 0;
    cs_singleTouchGestures[0].endPointArea.leftUpY = CS_ZONE_LEFT_TOP_Y-100;
    cs_singleTouchGestures[0].endPointArea.rightBottomX = CS_ZONE_RIGHT_BOTTOM_X;
    cs_singleTouchGestures[0].endPointArea.rightBottomY = CS_ZONE_RIGHT_BOTTOM_Y;
    cs_singleTouchGestures[0].keyId = KEY_CS_GESTURE_SWIPE_LEFT;
    cs_singleTouchGestures[0].needSendKeyDown = false;
    cs_singleTouchGestures[0].isLongTapGesture = false;
    cs_singleTouchGestures[0].isDetectOnTouchUp = true;


    // left to right swipe
    cs_tempTouchAcc.pointsCount = 2;
    cs_tempTouchAcc.points[0].x = 0;
    cs_tempTouchAcc.points[0].y = 0;
    cs_tempTouchAcc.points[1].x = 100;
    cs_tempTouchAcc.points[1].y = 0;

    cs_singleTouchGestures[1].touchAccPointsCount = cs_tempTouchAcc.pointsCount;
    memcpy(cs_singleTouchGestures[1].touchAccPoints, cs_tempTouchAcc.points,\
        cs_tempTouchAcc.pointsCount * sizeof(input_point_t));

    normalize_size(&cs_tempTouchAcc);
    normalize_spacing(&cs_singleTouchGestures[1], &cs_tempTouchAcc);
    normalize_center(&cs_singleTouchGestures[1]);
    // gesture constraints
    cs_singleTouchGestures[1].minDx = 72;//10%
    cs_singleTouchGestures[1].minDy = 0;
    cs_singleTouchGestures[1].maxDx = 0;
    cs_singleTouchGestures[1].maxDy = 0;
    cs_singleTouchGestures[1].startPointArea.leftUpX = CS_ZONE_LEFT_TOP_X;
    cs_singleTouchGestures[1].startPointArea.leftUpY = CS_ZONE_LEFT_TOP_Y;
    cs_singleTouchGestures[1].startPointArea.rightBottomX = CS_ZONE_RIGHT_BOTTOM_X;
    cs_singleTouchGestures[1].startPointArea.rightBottomY = CS_ZONE_RIGHT_BOTTOM_Y;
    cs_singleTouchGestures[1].minTime = 50;
    cs_singleTouchGestures[1].maxTime = 3000;
    cs_singleTouchGestures[1].endPointArea.leftUpX = 0;
    cs_singleTouchGestures[1].endPointArea.leftUpY = CS_ZONE_LEFT_TOP_Y - 100;
    cs_singleTouchGestures[1].endPointArea.rightBottomX = CS_ZONE_RIGHT_BOTTOM_X;
    cs_singleTouchGestures[1].endPointArea.rightBottomY = CS_ZONE_RIGHT_BOTTOM_Y;
    cs_singleTouchGestures[1].keyId = KEY_CS_GESTURE_SWIPE_RIGHT;
    cs_singleTouchGestures[1].needSendKeyDown = false;
    cs_singleTouchGestures[1].isLongTapGesture = false;
    cs_singleTouchGestures[1].isDetectOnTouchUp = true;

    cs_singleTouchGesturesCount = 2;


    //*************************************************************

    cs_gesturesCount = 0;
    for (i = 0; i < cs_singleTouchGesturesCount; i++,cs_gesturesCount++) {
        cs_Gestures[cs_gesturesCount].status = GESTURE_STATUS_VALID;
        cs_Gestures[cs_gesturesCount].singleGesture = &cs_singleTouchGestures[i];
        cs_Gestures[cs_gesturesCount].dtGesture = 0;
    }
}

/*
 * Init capsense gesture recognition engine
 */
void cs_init_engine(void) {

    gesture_slot_t* cslot;
    int i;

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        cslot = &capsense_slots[i];
        cslot->status = SLOT_STATUS_NOT_USED;
        cslot->trackingId = -1;
        cslot->changed = false;
    }

    csArea.leftUpX = CS_ZONE_LEFT_TOP_X;
    csArea.leftUpY = CS_ZONE_LEFT_TOP_Y;
    csArea.rightBottomX = CS_ZONE_RIGHT_BOTTOM_X;
    csArea.rightBottomY = CS_ZONE_RIGHT_BOTTOM_Y;

    for (i = 0; i < MAX_SLOTS_RECOGNITION; i++) {
        cs_inputTouches[i].pointsCount = 0;
    }

    cs_globalStatus.gestureType = GESTURE_TYPE_SINGLE_OR_DOUBLE;
    cs_globalStatus.compareResult = COMPARE_RESULT_NOT_ACTIVE;

}

/***********************************************************
 * Gesture recognition engine
 ***********************************************************/
/*
 * Normalize size of user input touch accumulator
 */
void normalize_size(touch_accumulator_t* ta) {
    long minX = LONG_MAX;
    long minY = LONG_MAX;
    long maxX = -LONG_MAX;
    long maxY = -LONG_MAX;
    long width;
    long height;
    long scale;

    int i;
    for (i = 0; i < ta->pointsCount; i++) {
        if (minX > ta->points[i].x)
            minX = ta->points[i].x;
        if (minY > ta->points[i].y)
            minY = ta->points[i].y;
        if (maxX < ta->points[i].x)
            maxX = ta->points[i].x;
        if (maxY < ta->points[i].y)
            maxY = ta->points[i].y;
    }
    // Calculate dimension
    width = maxX - minX;
    height = maxY - minY;
    //find out scale
    scale = (width > height) ? width : height;
    if (scale <= 0)
        return;

    for (i = 0; i < ta->pointsCount; i++) {
        ta->points[i].x = (ta->points[i].x * GESTURE_SCALE_FACTOR) / scale;
       ta->points[i].y = (ta->points[i].y * GESTURE_SCALE_FACTOR) / scale;
    }
}

/*
 *  Return the length of user input touch accumulator
 */
long get_gesture_length(touch_accumulator_t * ta) {
    long len;
    input_point_t * startPt;
    int i;
    if (ta->pointsCount <= 1)
        return 0;

    len = 0;
    startPt = &ta->points[0];
    for (i = 1; i < ta->pointsCount; i++) {
        input_point_t * endPt;
        long dx, dy;

        endPt = &ta->points[i];

        dx = endPt->x - startPt->x;
        dy = endPt->y - startPt->y;
        len += sqrt_l(dx * dx + dy * dy);
        startPt = endPt;
    }
    return len;
}

/*
 * Normalize space between points in touch accumulator,
 * convert it to single_gesture_t struct
 */
void normalize_spacing(single_gesture_t *newGesture, touch_accumulator_t * oldGesture) {
    input_point_t *startPt;
    input_point_t *endPt;
    long newSegmentLen;
    long endOldDist = 0;
    long startOldDist = 0;
    long newDist = 0;
    long currSegmentLen = 0;
    int currentPoint = 0;

    if (newGesture == NULL || oldGesture == NULL)
        return;

    newGesture->pointsCount = 0;
    newSegmentLen = get_gesture_length(oldGesture);
    if (oldGesture->pointsCount <= 1 || newSegmentLen == 0)
        return;
    newSegmentLen = newSegmentLen / (POINTS_PER_GESTURE - 1);

    // add first point
    newGesture->points[0] = oldGesture->points[0];
    newGesture->pointsCount = 1;

    startPt = &oldGesture->points[0];
    endPt = &oldGesture->points[0];

    for (;;) {
        long excess = endOldDist - newDist;
        //we have accumulated enough length, add a point
        if (excess >= newSegmentLen) {
            input_point_t newPt;
            newDist += newSegmentLen;
            newPt.x = ((endPt->x - startPt->x) * (newDist - startOldDist)) / currSegmentLen
                    + startPt->x;
            newPt.y = ((endPt->y - startPt->y) * (newDist - startOldDist)) / currSegmentLen
                    + startPt->y;

		newGesture->points[newGesture->pointsCount++] = newPt;
            if (newGesture->pointsCount == POINTS_PER_GESTURE) {
                break;
            }
        } else {
            long dx, dy;
            if (currentPoint == oldGesture->pointsCount)
                break; // no more data

            // store the start of the current segment
            startPt = endPt;
            endPt = &oldGesture->points[currentPoint]; // get next point
            currentPoint++;

            dx = endPt->x - startPt->x;
            dy = endPt->y - startPt->y;

            // start accumulated distance
            // at the begining of thre segment
            startOldDist = endOldDist;
            // add the length of the current segment to the
            // total acumulated length
            currSegmentLen = sqrt_l(dx * dx + dy * dy);
            endOldDist += currSegmentLen;
        }
    }

    // due to floating point error we may miss the last
    // point of the gesture

    if (newGesture->pointsCount < POINTS_PER_GESTURE) {
        newGesture->points[newGesture->pointsCount] = *endPt;
        newGesture->pointsCount++;
    }
}

/*
 * Normalize gesture center point, move it to (0,0)
 */
void normalize_center(single_gesture_t * sg) {
    long centerX = 0;
    long centerY = 0;

    //calculate center of the gesture
    int i;
    for (i = 0; i < sg->pointsCount; i++) {
        centerX += sg->points[i].x;
        centerY += sg->points[i].y;
    }
    //calculate centroid
    if (sg->pointsCount <= 0)
        return; //empty gesture

    centerX /= sg->pointsCount;
    centerY /= sg->pointsCount;

    for (i = 0; i < sg->pointsCount; i++) {
        sg->points[i].x -= centerX;
        sg->points[i].y -= centerY;
    }

}

/*
 * Calculate correlation for two gestures
 */
long gesture_dot_product(single_gesture_t * gesture1, single_gesture_t * gesture2) {
    int i1;
    long dotProduct = 0;

    if (gesture1->pointsCount != gesture2->pointsCount) {
        LOG_V("Error, gesture1 and gesture2 sizes are different!\n");
        return 0;
    }

    for (i1 = 0; i1 < gesture1->pointsCount; i1++) {
        dotProduct += (long) (gesture1->points[i1].x * gesture2->points[i1].x)
                + (long) (gesture1->points[i1].y * gesture2->points[i1].y);
    }
    return dotProduct;
}

/*
 * Match two gestures using correlation formula
 */
long match(single_gesture_t * gesture1, single_gesture_t * gesture2) {
    long xx, score, score3, score4;
    score = gesture_dot_product(gesture1, gesture2) * 100;

    if (score == 0) {
        LOG_V("match: score <= 0.0\n");
        return 0;
    }

    score3 = gesture_dot_product(gesture1, gesture1);
    score4 = gesture_dot_product(gesture2, gesture2);
    xx = sqrt_l(score3) * sqrt_l(score4);
    score /= xx;
	return score;
}

/*
 * Compare user input with gesture
 */
long compare_gestures(touch_accumulator_t * gesture1, single_gesture_t * inputGesture,
        single_gesture_t * gesture2, compare_gesture_mode_t mode) {
    int deltaX, deltaY;
    unsigned long gestureTime;
    long result;

    LOG_D("Compare gesture with action ID - %d\n", gesture2->keyId);

    if (!gesture2->isLongTapGesture && mode == COMPARE_GESTURE_LONG_TOUCH){
        return 0;
	}

    if(gesture2->isDetectOnTouchUp && mode == COMPARE_GESTURE_ALL_MOVING){
        LOG_D("Don't recognize gesture, it is not touch up yet!!!\n");
        return 0;
    }

    if (!is_inside_area(&gesture1->startPoint, &gesture2->startPointArea)) {
        LOG_D("Start point out of gesture start area, x- %lu,  y- %lu\n", gesture1->startPoint.x,
                gesture1->startPoint.y);
        return GESTURE_COMPARE_NOT_GESTURE;
    }

    if (mode == COMPARE_GESTURE_ALL || mode == COMPARE_GESTURE_LONG_TOUCH
            || mode == COMPARE_GESTURE_ALL_MOVING) {
        if (gesture2->endPointArea.leftUpX != -1) {
            if (!is_inside_area(&gesture1->endPoint, &gesture2->endPointArea)) {
                LOG_D("end point outside end point area\n");
                return 0;
            }
        }

        deltaX = abs(gesture1->maxX - gesture1->minX);
        if (gesture2->minDx > 0 && gesture2->minDx > deltaX) {
            LOG_D("DeltaX is to small\n");
            return 0;
        }

        if (gesture2->maxDx > 0 && gesture2->maxDx < deltaX) {
            LOG_D("DeltaX is to big\n");
            return GESTURE_COMPARE_NOT_GESTURE;
        }

        deltaY = abs(gesture1->maxY - gesture1->minY);
        if (gesture2->minDy > 0 && gesture2->minDy > deltaY) {
            LOG_D("DeltaY is to small\n");
            return 0;
        }

        if (gesture2->maxDy > 0 && gesture2->maxDy < deltaY) {
            LOG_D("DeltaY is to big\n");
            return GESTURE_COMPARE_NOT_GESTURE;
        }

        gestureTime = tv2ms(&gesture1->endTime) - tv2ms(&gesture1->startTime);
        if (gesture2->minTime > 0 && gesture2->minTime > gestureTime) {
            LOG_D("Gesture time too small: gesture time - %lu, reference min value - %d\n",
                    gestureTime, gesture2->minTime);
            return 0;
        }

        if (gesture2->maxTime > 0 && gesture2->maxTime < gestureTime) {
            LOG_D("Gesture time too big: gesture time - %lu, reference max value - %d\n",
                    gestureTime, gesture2->maxTime);
            return GESTURE_COMPARE_NOT_GESTURE;
        }

    }
    if (gesture2->touchAccPointsCount == 0) {
        if (mode == COMPARE_GESTURE_LONG_TOUCH || mode == COMPARE_GESTURE_ALL) {
            LOG_D("Don't check template, It is a gesture\n");
            return 100;
        }
        return 0;
    }

    result = match(inputGesture, gesture2);
    LOG_D("compare result - %lu\n", result);
    return result;
}

/*
 * Calculate square root without float point operation
 */
long sqrt_l(long x1) {
    int count = 9;
    long r1, r2;
    long x = x1;
    int i;

    if (x1 <= 0)
        return 0;
    r2 = 0;
    if (x <= 1800) {
        r1 = 30;
        count = 4;
    } else {
        r1 = 500;
        count = 5;
    }

    for (i = 0; i < count; i++) {
        r2 = (r1 + x / r1) / 2;
        r1 = r2;
    }
    return r2;
}

/*
 * Convert struct timeval to ms(milliseconds)
 */
unsigned long int tv2ms(struct timeval *a) {
    return ((a->tv_sec * 1000) + (a->tv_usec / 1000));
}

/*
 * Check if point is inside area
 */
bool is_inside_area(input_point_t * p, rect_area_t * a) {
    return (p->x >= a->leftUpX && p->x <= a->rightBottomX && p->y >= a->leftUpY
        && p->y <= a->rightBottomY);
}

/*
 * Check if point is inside one of the extended areas.
 */
bool is_inside_extended_areas(input_point_t * point) {
    return (is_inside_area(point, &topArea) || is_inside_area(point, &bottomArea));
}

/*
 * Copy one touch_accumulator_t to another
 */
void copy_touch_accumulator(touch_accumulator_t *src, touch_accumulator_t *dest) {
    int i;
    dest->endTime = src->endTime;
    dest->maxX = src->maxX;
    dest->maxY = src->maxY;
    dest->minX = src->minX;
    dest->minY = src->minY;
    dest->pointsCount = src->pointsCount;
    dest->startPoint = src->startPoint;
    dest->startTime = src->startTime;
    dest->endPoint = src->endPoint;

    for (i = 0; i < dest->pointsCount; i++) {
        dest->points[i] = src->points[i];
    }
}

/*
 * Fill gestureInputDev fields from gesture data
 */
void gesture_input_dev_fill(void) {
    int i;

    memset(gestureInputDev->keybit, 0, BITS_TO_LONGS(KEY_CNT));

    for(i = 0; i < gesturesCount; i++) {
        if (gestures[i].singleGesture != NULL) {
            __set_bit(gestures[i].singleGesture->keyId, gestureInputDev->keybit);
        } else {
            __set_bit(gestures[i].dtGesture->keyId, gestureInputDev->keybit);
        }
    }
    __set_bit(KEY_FS_GESTURE_LOCK, gestureInputDev->keybit);
    __set_bit(KEY_FS_GESTURE_UNLOCK, gestureInputDev->keybit);

	// add capsense gesture key IDs
    __set_bit(KEY_CS_GESTURE_SWIPE_LEFT, gestureInputDev->keybit);
    __set_bit(KEY_CS_GESTURE_SWIPE_RIGHT, gestureInputDev->keybit);

    // add BS gesture key IDs
    /*    __set_bit(KEY_BS_GESTURE_SWIPE_RIGHT, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_SWIPE_LEFT, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_SINGLE_TAP, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_DOUBLE_TAP, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_LONG_TAP, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_RIGHT_LEFT_RIGHT, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_LEFT_RIGHT_LEFT, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_SCROLL_LEFT, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_SCROLL_RIGHT, gestureInputDev->keybit);*/

    __set_bit(KEY_BS_GESTURE_LOCK, gestureInputDev->keybit);
    __set_bit(KEY_BS_GESTURE_UNLOCK, gestureInputDev->keybit);

#ifdef BS_FORWARD_RAW_KEY_EVENTS
    __set_bit(KEY_BS_KEY0, gestureInputDev->keybit);
    __set_bit(KEY_BS_KEY1, gestureInputDev->keybit);
    __set_bit(KEY_BS_KEY2, gestureInputDev->keybit);
    __set_bit(KEY_BS_KEY3, gestureInputDev->keybit);
    __set_bit(KEY_BS_KEY4, gestureInputDev->keybit);
    __set_bit(KEY_BS_KEY5, gestureInputDev->keybit);
    __set_bit(KEY_BS_KEY6, gestureInputDev->keybit);
    __set_bit(KEY_BS_KEY7, gestureInputDev->keybit);
#endif
}

/*
 * Register gesture input device
 */
static int gesture_ts_init(void) {
    int error;

    LOG_V("gesture_ts_init");
    gestureInputDev = input_allocate_device();
    if (!gestureInputDev) {
        LOG_E("Failed to allocate memory\n");
        return -ENOMEM;

    }

    gestureInputDev->name = "gestures";
    gestureInputDev->id.bustype = BUS_VIRTUAL;
    gestureInputDev->dev.parent = NULL;

    //TODO get key IDs from gestures
    __set_bit(EV_KEY, gestureInputDev->evbit);

    gesture_input_dev_fill();

    error = input_register_device(gestureInputDev);
    if (error) {
        input_free_device(gestureInputDev);
        gestureInputDev = NULL;
        LOG_E("Failed to register device\n");
    }

    return error;
}

/*
 * Unregister gesture input device
 */
static void gesture_ts_exit(void) {
    LOG_V("gesture_ts_exit\n");
    if (gestureInputDev != NULL) {
        input_unregister_device(gestureInputDev);
        gestureInputDev = NULL;
    }
}

/*
 * Report key down, up events using gesture input device
 */
void report_key(unsigned int key) {
    report_key_down(key);
    report_key_up(key);
}

/*
 * Report key down using gesture input device
 */
void report_key_down(unsigned int key) {
    if (gestureInputDev != NULL) {
        input_report_key(gestureInputDev, key, 1);
        input_sync(gestureInputDev);
    }
}

/*
 * Report key up using gesture input device
 */
void report_key_up(unsigned int key) {
    if (gestureInputDev != NULL) {
        input_report_key(gestureInputDev, key, 0);
        input_sync(gestureInputDev);
    }
}

/*
 * Main touch handle function, recognize gesture,
 * return filter result
 */
bool handle_touch() {
    gesture_slot_t* slot;
    bool result = false;
    int i;
    bool newEvent = false;

    update_gesture_type();
    update_available_gestures();

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        slot = &slots[i];
        if (slot->status == SLOT_STATUS_NOT_USED && slot->changed) {
            //blockFsEvents = false;
	    // new touch
            LOG_D("new touch in  slot %d  X=%d, y=%d, tracking ID = %d\n", i, slot->positionX,
                    slot->positionY, slot->trackingId);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                add_new_point(i);
            }
            slot->changed = 0;
            slot->status = SLOT_STATUS_NEW_TOUCH;
        } else if ((slot->status == SLOT_STATUS_NEW_TOUCH || slot->status == SLOT_STATUS_CHANGED)
                && slot->changed) {
            //change
            LOG_D("change in current slot %d  X=%d, y=%d, tracking ID = %d\n", i, slot->positionX,
                    slot->positionY, slot->trackingId);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                add_new_point(i);
            }
            slot->status = SLOT_STATUS_CHANGED;
            slot->changed = 0;
        } else if (slot->status == SLOT_STATUS_TOUCH_UP && slot->changed) {
            // this is touch up
            LOG_D("touch up detected for slot - %d \n", i);
            newEvent = true;
            if (i < MAX_SLOTS_RECOGNITION) {
                add_new_point(i);
            }
            slot->status = SLOT_STATUS_NOT_USED;
            slot->trackingId = -1;
            slot->changed = false;
        }

    }

    if (!newEvent) {
        // return if don't have new events
        LOG_V("No new events, globalStatus.compareResult - %d\n", globalStatus.compareResult);
        switch (globalStatus.compareResult) {
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
    update_gesture_type();
// ***********************************
//   Long touch support
    if (globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
            && slots[0].status == SLOT_STATUS_NEW_TOUCH) {
        // one new touch start long touch timer
        int ret;
        ret = mod_timer(&longTouchTimer.timer, jiffies + msecs_to_jiffies(LONG_TOUCH_MIN_TIME));
        longTouchTimer.trackingId = slots[0].trackingId;
        longTouchTimer.count = 0;
    } else if (globalStatus.gestureType != GESTURE_TYPE_SINGLE_OR_DOUBLE) {
        // //reset timer if more than one touch or touch up happened
        longTouchTimer.trackingId = -1;
    }
// Long touch support
//*****************************************

    switch (globalStatus.gestureType) {
    case GESTURE_TYPE_SINGLE_OR_DOUBLE:
    case GESTURE_TYPE_SINGLE:
    case GESTURE_TYPE_DOUBLE:
        switch (globalStatus.compareResult) {
        case COMPARE_RESULT_GESTURE:
            // notify and filter it
		    result = true;
            if (globalStatus.action < 0) {
                // already sent
                break;
            }
            if (globalStatus.needSendKeyDown) {
                globalStatus.delayedKeyUP = globalStatus.action;
                LOG_V("send single key down\n");
                report_key_down(globalStatus.action);
                globalStatus.action = -1;
            }
            break;
        case COMPARE_RESULT_NOT_GESTURE:
        case COMPARE_RESULT_NOT_ACTIVE:
        case COMPARE_RESULT_NOT_RECOGNIZED:
            result = false;
            break;
        case COMPARE_RESULT_NOT_GESTURE_FILTERED:
            result = true;
            break;
        case COMPARE_RESULT_PART_OF_GESTURE:
            // probably gesture
            // moved to screen area, disable double touch gestures
            if (get_valid_gestures_count() == 0) {
                globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE_FILTERED;
            }
		result = true;
            break;
        }
        break;
    case GESTURE_TYPE_NOT_GESTURE:
        switch (globalStatus.compareResult) {
        case COMPARE_RESULT_PART_OF_GESTURE:
        case COMPARE_RESULT_GESTURE:
        case COMPARE_RESULT_NOT_GESTURE_FILTERED:
            result = true;
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
    if (globalStatus.compareResult == COMPARE_RESULT_GESTURE
            && (globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
                    || globalStatus.gestureType == GESTURE_TYPE_SINGLE)
            && slots[0].status == SLOT_STATUS_NOT_USED) {
        if (globalStatus.needSendKeyDown && globalStatus.delayedKeyUP >= 0) {
            LOG_V("send  key up\n");
            report_key_up(globalStatus.delayedKeyUP);
            globalStatus.delayedKeyUP = -1;
        } else {
            report_key(globalStatus.action);
        }

    }

//2. double touch gesture
    if (globalStatus.compareResult == COMPARE_RESULT_GESTURE && globalStatus.delayedKeyUP >= 0
            && globalStatus.gestureType == GESTURE_TYPE_DOUBLE
            && (slots[0].status == SLOT_STATUS_NOT_USED
                    || slots[1].status == SLOT_STATUS_NOT_USED) && capsenseSlots == DOUBLE_TOUCH_SLOT ) {
          if (globalStatus.needSendKeyDown && globalStatus.delayedKeyUP >= 0) {
            LOG_V("send  key up\n");
            report_key_up(globalStatus.delayedKeyUP);
            globalStatus.delayedKeyUP = -1;
        } else {
            report_key(globalStatus.action);
        }
    }
#if 0
//3. lock gesture: some garbage touches
    if (bsStatus.fs_lock_enabled && (get_used_slots_count() >=3) && (globalStatus.compareResult == COMPARE_RESULT_NOT_GESTURE)) {
        mod_timer(&fs_lock_event_timer, jiffies+msecs_to_jiffies(LOCK_EVENT_TIMER));
    }

    if (bsStatus.fs_unlock_enabled && bsStatus.fs_locked) {
	report_key(KEY_FS_GESTURE_UNLOCK);
	bsStatus.fs_locked = false;
	blockFsEvents = true;
    }
#endif



//*******************************************************************
// make all gestures available if there are no touches.
    if (get_used_slots_count() == 0) {
        for (i = 0; i < gesturesCount; i++) {
            gestures[i].status = GESTURE_STATUS_VALID;

        }
        globalStatus.gestureType = GESTURE_TYPE_SINGLE_OR_DOUBLE;
        globalStatus.compareResult = COMPARE_RESULT_NOT_ACTIVE;
        globalStatus.action = -1;
        globalStatus.delayedKeyUP = -1;
	 capsenseSlots = 0;
    }
//********************************************************************

    LOG_V("handletouch filter result - %d\n", result);
    LOG_V("compare_result = %d\n", globalStatus.compareResult);
    LOG_V("gesture type = %d\n", globalStatus.gestureType);
    return result;
}

/*
 * Add new point, call gesture recognition if necessary
 * Update gesture compare_result.
 */
void add_new_point(int inputId) {
    gesture_slot_t * slot;
    slot = &slots[inputId];
    switch (globalStatus.compareResult) {
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
            if (globalStatus.compareResult == COMPARE_RESULT_PART_OF_GESTURE) {
                // cann't accept new touch in this state
                globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE_FILTERED;
                break;
            }
            // add first point
            inputTouches[inputId].pointsCount = 1;
            inputTouches[inputId].points[0].x = slot->positionX;
            inputTouches[inputId].points[0].y = slot->positionY;
            inputTouches[inputId].startPoint.x = slot->positionX;
            inputTouches[inputId].startPoint.y = slot->positionY;
            inputTouches[inputId].endPoint = inputTouches[inputId].startPoint;
            inputTouches[inputId].minX = slot->positionX;
            inputTouches[inputId].minY = slot->positionY;
            inputTouches[inputId].maxX = inputTouches[inputId].minX;
            inputTouches[inputId].maxY = inputTouches[inputId].minY;
            inputTouches[inputId].startTime = slot->time;
            inputTouches[inputId].endTime = slot->time;
            if (!is_inside_extended_areas(&inputTouches[inputId].startPoint)) {
                globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
            } else {
                globalStatus.compareResult = COMPARE_RESULT_NOT_RECOGNIZED;
            }

            break;
        case SLOT_STATUS_NEW_TOUCH:
        case SLOT_STATUS_CHANGED:
        case SLOT_STATUS_TOUCH_UP:
            // add new point
            if (inputTouches[inputId].pointsCount >= TOUCH_ACC_MAX_POINTS_COUNT) {
                // too many points, break
                //TODO something
                LOG_E("Too many points...\n");
                break;
            }
            inputTouches[inputId].points[inputTouches[inputId].pointsCount].x = slot->positionX;
            inputTouches[inputId].points[inputTouches[inputId].pointsCount].y = slot->positionY;
            inputTouches[inputId].pointsCount++;
            inputTouches[inputId].endTime = slot->time;
            inputTouches[inputId].endPoint.x = slot->positionX;
            inputTouches[inputId].endPoint.y = slot->positionY;

            // update min and max values
            if (slot->positionX < inputTouches[inputId].minX) {
                inputTouches[inputId].minX = slot->positionX;
            } else if (slot->positionX > inputTouches[inputId].maxX) {
                inputTouches[inputId].maxX = slot->positionX;
            }

            if (slot->positionY < inputTouches[inputId].minY) {
                inputTouches[inputId].minY = slot->positionY;
            } else if (slot->positionY > inputTouches[inputId].maxY) {
                inputTouches[inputId].maxY = slot->positionY;
            }

            if (slot->status == SLOT_STATUS_TOUCH_UP) {
                all_gesture_compare_result_t result;
                result = compare_all_gestures(COMPARE_GESTURE_ALL);
                if (result.keyId >= 0) {
                    globalStatus.action = result.keyId;
                    globalStatus.needSendKeyDown = result.needSendKeyDown;
                    globalStatus.compareResult = COMPARE_RESULT_GESTURE;
                }

            }
            // check last point area
            if (is_inside_extended_areas(
                    &inputTouches[inputId].points[inputTouches[inputId].pointsCount - 1])) {
                // we are in extended area now
                if (!is_inside_extended_areas(&inputTouches[inputId].startPoint)) {
                    // started from screen area and moved to extended area, not a gesture
                    globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                }
            } else {
                if (is_inside_extended_areas(&inputTouches[inputId].startPoint)) {
                    // started from extended areas and moved to screen area.
                    // need to check for gesture
                    LOG_V("started in ext areas and moved to screen area\n");
                    if (globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE) {
			globalStatus.gestureType = GESTURE_TYPE_SINGLE;
                        disable_dt_gesture();
                    }
                    LOG_V("Moved to screen , gesture type- %d\n", globalStatus.gestureType);

                    if (globalStatus.compareResult == COMPARE_RESULT_PART_OF_GESTURE) {
                        all_gesture_compare_result_t result;
                        result = compare_all_gestures(COMPARE_GESTURE_ALL_MOVING);
                        if (result.keyId >= 0) {
                            globalStatus.action = result.keyId;
                            globalStatus.needSendKeyDown = result.needSendKeyDown;
                            globalStatus.compareResult = COMPARE_RESULT_GESTURE;
                        }
                    } else if (globalStatus.compareResult == COMPARE_RESULT_NOT_RECOGNIZED) {
                        all_gesture_compare_result_t result;
                        result = compare_all_gestures(COMPARE_GESTURE_AND_START_POINT);
                        if (result.keyId >= 0) {
                            globalStatus.action = result.keyId;
                            globalStatus.needSendKeyDown = result.needSendKeyDown;
                            globalStatus.compareResult = COMPARE_RESULT_PART_OF_GESTURE;
			} else {
                            globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                        }
                    }
                } else {
                    // started from screen area, it is not a gesture
                    globalStatus.compareResult = COMPARE_RESULT_NOT_GESTURE;
                }
            }
            break;
        }
        break;
    }
    LOG_V("add new point result input0 - %d\n", globalStatus.compareResult);
}

/*
 * Update type of gesture that we will recognize
 */
void update_gesture_type(void) {
    int i;
    gesture_slot_t * sl;
    int slotsCount = 0;
    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        sl = &slots[i];
        if (sl->status != SLOT_STATUS_NOT_USED) {
            slotsCount++;
        } else if (sl->changed) {
            slotsCount++;
        }
    }
    switch (globalStatus.compareResult) {
    case COMPARE_RESULT_NOT_GESTURE:
	break;
    case COMPARE_RESULT_GESTURE:
        //don't change gesture type if we already recognized gesture
        break;
    case COMPARE_RESULT_NOT_ACTIVE:
    case COMPARE_RESULT_NOT_RECOGNIZED:

        if (slotsCount == MAX_SLOTS_RECOGNITION && capsenseSlots == DOUBLE_TOUCH_SLOT ) {
            globalStatus.gestureType = GESTURE_TYPE_DOUBLE;
        } else if (slotsCount > MAX_SLOTS_RECOGNITION ) {
            globalStatus.gestureType = GESTURE_TYPE_NOT_GESTURE;
        }
        break;
    case COMPARE_RESULT_PART_OF_GESTURE:
        // we should have decision about gesture type here
        if ((globalStatus.gestureType == GESTURE_TYPE_DOUBLE
                && slotsCount != MAX_SLOTS_RECOGNITION)
                || (globalStatus.gestureType == GESTURE_TYPE_SINGLE && slotsCount != 1)) {
            globalStatus.gestureType = GESTURE_TYPE_NOT_GESTURE;
        }
        break;

    case COMPARE_RESULT_NOT_GESTURE_FILTERED:
        break;
    }
}

/*
 * Long touch callback function
 */
void long_touch_callback(unsigned long data) {
    LOG_V("long touch callback \n");
    if (longTouchTimer.trackingId < 0) {
        LOG_V("timer was reset\n");
        return;
    }
    if (get_used_slots_count() == 1 && longTouchTimer.trackingId == slots[0].trackingId) {
        if (is_inside_extended_areas(&inputTouches[0].startPoint)) {
            struct timespec ts;
            all_gesture_compare_result_t result;
            // update end time
            ktime_get_ts(&ts);
            inputTouches[0].endTime.tv_sec = ts.tv_sec;
            inputTouches[0].endTime.tv_usec = ts.tv_nsec / NSEC_PER_USEC;
            result = compare_all_gestures(COMPARE_GESTURE_LONG_TOUCH);
            if (result.keyId >= 0) {
                globalStatus.compareResult = COMPARE_RESULT_GESTURE;
                globalStatus.needSendKeyDown = result.needSendKeyDown;
                globalStatus.action = result.keyId;

                if (globalStatus.needSendKeyDown) {
                    globalStatus.delayedKeyUP = globalStatus.action;
                    report_key_down(globalStatus.action);
                    globalStatus.action = -1;
                }
            } else {
                // increase timer
                if (longTouchTimer.count < LONG_TOUCH_MAX_COUNT) {
                    longTouchTimer.count++;
                    mod_timer(&longTouchTimer.timer,
                            jiffies + msecs_to_jiffies(LONG_TOUCH_MIN_TIME));
                }
            }
        }
    }
}

/*
 * Update available gesture list based on current type
 */
void update_available_gestures(void) {
    int i;
    bool cancelST = false;
    bool cancelDT = false;
    switch (globalStatus.gestureType) {
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

    for (i = 0; i < gesturesCount; i++) {
        if (cancelST && gestures[i].singleGesture != 0
                && gestures[i].status != GESTURE_STATUS_RECOGNIZED) {
            gestures[i].status = GESTURE_STATUS_NOT_VALID;

        } else if (cancelDT && gestures[i].dtGesture != 0
                && gestures[i].status != GESTURE_STATUS_RECOGNIZED) {
            gestures[i].status = GESTURE_STATUS_NOT_VALID;
        }
    }
}


/*
 * Disable all double touch gestures
 */
void disable_dt_gesture(void) {
    int i;
    for (i = 0; i < gesturesCount; i++) {
        if (gestures[i].dtGesture != 0) {
            gestures[i].status = GESTURE_STATUS_NOT_VALID;
        }
    }
}

/*
 * Return count of all valid gestures
 */
int get_valid_gestures_count(void) {
    int i, count;
    count = 0;
    for (i = 0; i < gesturesCount; i++) {
        if (gestures[i].status != GESTURE_STATUS_NOT_VALID) {
            count++;
        }
    }
    return count;
}
#if 0
void fs_lock_event_callback(unsigned long data)
{
    if (bsStatus.fs_lock_enabled && (get_used_slots_count() >=3) && (globalStatus.compareResult == COMPARE_RESULT_NOT_GESTURE)) {
        report_key(KEY_FS_GESTURE_LOCK);
        blockFsEvents = false;
    }
}
#endif

/*
 * Compare user input with all valid gestures
 */
all_gesture_compare_result_t compare_all_gestures(compare_gesture_mode_t mode) {
    single_gesture_t tempGesture, tempGesture2;
    all_gesture_compare_result_t cResult;
    bool separateKeyUPKeyDown = false;
    long result;
    int i;
    int maxGestureValue = 0;
    int maxGestureKeyId = -1;
    copy_touch_accumulator(&inputTouches[0], &tempTouchAcc);
    normalize_size(&tempTouchAcc);
    normalize_spacing(&tempGesture, &tempTouchAcc);
    normalize_center(&tempGesture);

    if (globalStatus.gestureType == GESTURE_TYPE_DOUBLE) {
        copy_touch_accumulator(&inputTouches[1], &tempTouchAcc2);
        normalize_size(&tempTouchAcc2);
        normalize_spacing(&tempGesture2, &tempTouchAcc2);
        normalize_center(&tempGesture2);
    }

    for (i = 0; i < gesturesCount; i++) {
        if (gestures[i].status == GESTURE_STATUS_VALID) {
            if (gestures[i].singleGesture != 0
                    && (globalStatus.gestureType == GESTURE_TYPE_SINGLE_OR_DOUBLE
                            || globalStatus.gestureType == GESTURE_TYPE_SINGLE)) {
                result = compare_gestures(&tempTouchAcc, &tempGesture, gestures[i].singleGesture,
                        mode);
                if (result == GESTURE_COMPARE_NOT_GESTURE) {
                    gestures[i].status = GESTURE_STATUS_NOT_VALID;
                    continue;
                }
                if (result > maxGestureValue) {
                    maxGestureValue = result;
                    maxGestureKeyId = gestures[i].singleGesture->keyId;
                    separateKeyUPKeyDown = gestures[i].singleGesture->needSendKeyDown;
                }
            }
            if (gestures[i].dtGesture != 0 && globalStatus.gestureType == GESTURE_TYPE_DOUBLE) {
                result = compare_dt_gestures(&tempTouchAcc, &tempTouchAcc2, &tempGesture,
                        &tempGesture2, gestures[i].dtGesture, mode);
                if (result > maxGestureValue) {
                    maxGestureValue = result;
                    maxGestureKeyId = gestures[i].dtGesture->keyId;
                    separateKeyUPKeyDown = gestures[i].dtGesture->gesture1.needSendKeyDown;
                }
            }
        }
    }
    LOG_D("Compare all gesture: max result - %d, key ID - %d\n", maxGestureValue, maxGestureKeyId);
    if (maxGestureValue > GESTURE_RECOGNITION_THRESHOLD) {
        // gesture is recognized
        cResult.keyId = maxGestureKeyId;
        cResult.needSendKeyDown = separateKeyUPKeyDown;
    } else {
        cResult.keyId = -1;
        cResult.needSendKeyDown = false;
    }
    return cResult;
}

/*
 * Compare user input with double touch gesture
 */
long compare_dt_gestures(touch_accumulator_t * input0, touch_accumulator_t * input1,
        single_gesture_t * inputGesture0, single_gesture_t * inputGesture1,
        double_touch_gesture_t * dtGesture, compare_gesture_mode_t mode) {

    long result01, result02, result11, result12;

    result01 = compare_gestures(input0, inputGesture0, &dtGesture->gesture1, mode);
    result12 = compare_gestures(input1, inputGesture1, &dtGesture->gesture2, mode);

    if ((mode != COMPARE_GESTURE_AND_START_POINT && result01 > GESTURE_RECOGNITION_THRESHOLD && result12 > GESTURE_RECOGNITION_THRESHOLD)
        || (mode == COMPARE_GESTURE_AND_START_POINT && (result01 > GESTURE_RECOGNITION_THRESHOLD || result12 > GESTURE_RECOGNITION_THRESHOLD))) {
        // it is double gesture
        if(mode == COMPARE_GESTURE_AND_START_POINT) {
            return max(result01, result12);
        } else {
            return min(result01, result12);
        }
    } else {
        result02 = compare_gestures(input0, inputGesture0, &dtGesture->gesture2, mode);
        result11 = compare_gestures(input1, inputGesture1, &dtGesture->gesture1, mode);

        if ((mode != COMPARE_GESTURE_AND_START_POINT && result02 > GESTURE_RECOGNITION_THRESHOLD && result11 > GESTURE_RECOGNITION_THRESHOLD)
            || (mode == COMPARE_GESTURE_AND_START_POINT && (result02 > GESTURE_RECOGNITION_THRESHOLD || result11 > GESTURE_RECOGNITION_THRESHOLD))) {
            // it is double gesture
            if(mode == COMPARE_GESTURE_AND_START_POINT) {
                return max(result02, result11);
            } else {
                return min(result02, result11);
            }
        }
    }
    //todo make gesture invalid if it definitely not gesture
    return 0;
}

/*
 * Get currently used slots count
 */
int get_used_slots_count(void) {
    int i;
    int slotsCount = 0;

    for (i = 0; i < MAX_SLOTS_COUNT; i++) {
        if (slots[i].status != SLOT_STATUS_NOT_USED) {
            slotsCount++;
        } else if (slots[i].changed) {
            slotsCount++;
        }
    }
    return slotsCount;
}

/*
 * Virtual input device init
 */
static int virtual_input_device_init(void) {
    int error;

    LOG_V("virtual_ts_init");
    virtualTPInputDev = input_allocate_device();
    if (!virtualTPInputDev) {
        LOG_E("Failed to allocate memory\n");
        return -ENOMEM;

    }

    virtualTPInputDev->name = "fs_virtual_ts";
    virtualTPInputDev->id.bustype = BUS_VIRTUAL;
    virtualTPInputDev->dev.parent = NULL;
    set_bit(EV_SYN, virtualTPInputDev->evbit);
    set_bit(EV_KEY, virtualTPInputDev->evbit);
    set_bit(EV_ABS, virtualTPInputDev->evbit);

    //set_bit(BTN_TOUCH, virtualTPInputDev->keybit);

    /*set_bit(KEY_BS_KEY0, virtualTPInputDev->keybit);
    set_bit(KEY_BS_KEY1, virtualTPInputDev->keybit);
    set_bit(KEY_BS_KEY2, virtualTPInputDev->keybit);
    set_bit(KEY_BS_KEY3, virtualTPInputDev->keybit);
    set_bit(KEY_BS_KEY4, virtualTPInputDev->keybit);
    set_bit(KEY_BS_KEY5, virtualTPInputDev->keybit);
    set_bit(KEY_BS_KEY6, virtualTPInputDev->keybit);
    set_bit(KEY_BS_KEY7, virtualTPInputDev->keybit);*/

    set_bit(INPUT_PROP_DIRECT, virtualTPInputDev->propbit);

    /* For multi touch */
    input_mt_init_slots(virtualTPInputDev, 10);

    input_set_abs_params(virtualTPInputDev, ABS_MT_POSITION_X, 0, TOP_ZONE_RIGHT_BOTTOM_X, 0, 0);
    input_set_abs_params(virtualTPInputDev, ABS_MT_POSITION_Y, 0,
            BOTTOM_ZONE_LEFT_TOP_Y, 0, 0);

    input_set_abs_params(virtualTPInputDev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(virtualTPInputDev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(virtualTPInputDev, ABS_MT_PRESSURE, 0, 255, 0, 0);

    error = input_register_device(virtualTPInputDev);
    if (error) {
        input_free_device(virtualTPInputDev);
        virtualTPInputDev = NULL;
        LOG_E("Failed to register device\n");
    }

    return error;
}

/*
 * Unregister virtual input device
 */
static void virtual_input_device_exit(void) {
    if (virtualTPInputDev != NULL) {
        input_unregister_device(virtualTPInputDev);
        virtualTPInputDev = NULL;
    }
}

MODULE_AUTHOR("Alexander Dubinin <Alexander.Dubinin@Symphonyteleca.com>");
MODULE_DESCRIPTION("Gesture recognition driver");
MODULE_LICENSE("GPL");

MODULE_DEVICE_TABLE(input, gesture_driver_ids);

module_init(gesture_driver_init);
module_exit(gesture_driver_exit);
