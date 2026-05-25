/**

 * @file task_odrive.c

 * @brief Poll encoder estimates from two separate ODrive boards on CAN.

 *

 * Hardware: 2x single-axis drives (e.g. MKS XDrive Mini), CAN node_id 0 and 1.

 * Each board only uses axis0; messages are addressed by node_id, not axis1.

 *

 * RX: ISR updates per-node snapshots (odrive_can_dma_on_rx_frame).

 * This task: copy snapshots, RTR refresh per stale drive, publish app_samples.

 */



#include "tasks/tasks.h"



#include "app_config.h"

#include "app_samples.h"

#include "app_time_us.h"

#include "fdcan.h"

#include "odrive_can_dma.h"



#include "FreeRTOS.h"

#include "task.h"



volatile uint32_t g_odrive_poll_count;

/** Bit i set when drive[i] (CAN node i) has a valid encoder snapshot. */

volatile uint32_t g_odrive_drive_valid_mask;



static const uint32_t s_drive_node_ids[APP_ODRIVE_DRIVE_COUNT] = {

    APP_ODRIVE_DRIVE0_NODE_ID,

    APP_ODRIVE_DRIVE1_NODE_ID,

};



static void odrive_drive_from_snapshot(uint32_t node_id, const ODriveCanDmaEncoderSnapshot *snap,

                                       app_odrive_drive_sample_t *out)

{

    out->node_id = node_id;

    out->valid = snap->valid;

    out->pos_turns = snap->encoder_pos_turns;

    out->vel_turns_s = snap->encoder_vel_turns_s;

    out->pos_counts = snap->encoder_pos_counts;

    out->last_update_ms = snap->last_update_ms;

}



void task_odrive(void *argument)

{

    (void)argument;



    uint32_t pub_seq = 0u;

    TickType_t wake = xTaskGetTickCount();

    const TickType_t period = pdMS_TO_TICKS(APP_ODRIVE_PERIOD_MS);



    for (;;) {

        vTaskDelayUntil(&wake, period);



        g_odrive_poll_count++;



        app_odrive_sample_t sample = {0};

        sample.t_us = app_time_us_now();

        sample.seq = ++pub_seq;



        uint32_t valid_mask = 0u;



        for (uint32_t drive_idx = 0u; drive_idx < APP_ODRIVE_DRIVE_COUNT; drive_idx++) {

            const uint32_t node_id = s_drive_node_ids[drive_idx];

            ODriveCanDmaEncoderSnapshot snap = {0};



            (void)odrive_can_dma_get_encoder_snapshot(node_id, &snap);



            if (!odrive_can_dma_is_encoder_fresh(node_id, APP_ODRIVE_ENCODER_STALE_MS)) {

                (void)odrive_can_dma_request_encoder_estimates(node_id);

            }



            odrive_drive_from_snapshot(node_id, &snap, &sample.drive[drive_idx]);



            if (sample.drive[drive_idx].valid) {

                valid_mask |= (1u << drive_idx);

            }

        }



        g_odrive_drive_valid_mask = valid_mask;

        app_samples_odrive_publish(&sample);

    }

}


