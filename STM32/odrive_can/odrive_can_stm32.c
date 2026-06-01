/**
 * @file odrive_can_stm32.c
 */

#include "odrive_can_stm32.h"

bool odrive_can_send(ODriveCanHalHandle *hcan, uint32_t node_id, ODriveCanMsg cmd, bool remote,
                     const uint8_t *data, uint8_t dlc)
{
    if (hcan == NULL) {
        return false;
    }
    const uint32_t id = odrive_can_std_id(node_id, cmd);
    return odrive_can_hal_tx(hcan, id, remote, data, dlc);
}

bool odrive_can_send_data(ODriveCanHalHandle *hcan, uint32_t node_id, ODriveCanMsg cmd,
                          const uint8_t *data, uint8_t dlc)
{
    return odrive_can_send(hcan, node_id, cmd, false, data, dlc);
}

bool odrive_can_send_rtr(ODriveCanHalHandle *hcan, uint32_t node_id, ODriveCanMsg cmd)
{
    return odrive_can_send(hcan, node_id, cmd, true, NULL, 8u);
}

bool odrive_can_receive(ODriveCanHalHandle *hcan, ODriveCanFrame *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t id = 0u;
    bool ext = false;
    bool rtr = false;
    uint8_t dlc = 0u;

    if (!odrive_can_hal_rx(hcan, &id, &ext, &rtr, out->data, &dlc)) {
        return false;
    }

    out->id = id;
    out->is_extended = ext;
    out->is_rtr = rtr;
    out->dlc = dlc;
    return true;
}
