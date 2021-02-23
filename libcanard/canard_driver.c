/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "canard_driver.h"
#include "canard.h"
#include "uavcan/equipment/esc/Status.h"
#include "uavcan/equipment/esc/RawCommand.h"
#include "uavcan/equipment/esc/RPMCommand.h"
#include "uavcan/protocol/param/GetSet.h"
#include "uavcan/protocol/GetNodeInfo.h"

#include "conf_general.h"
#include "app.h"
#include "comm_can.h"
#include "commands.h"
#include "mc_interface.h"
#include "hw.h"
#include "timeout.h"
#include "terminal.h"
#include "mempools.h"

// Constants
#define CAN_APP_NODE_NAME								"org.vesc." HW_NAME

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE			((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE		0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID				1

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE					7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID					341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE			0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK							0
#define UAVCAN_NODE_HEALTH_WARNING						1
#define UAVCAN_NODE_HEALTH_ERROR						2
#define UAVCAN_NODE_HEALTH_CRITICAL						3

#define UAVCAN_NODE_MODE_OPERATIONAL					0
#define UAVCAN_NODE_MODE_INITIALIZATION					1

#define UNIQUE_ID_LENGTH_BYTES							12

#define STATUS_MSGS_TO_STORE							10

#define AP_MAX_NAME_SIZE								16

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

#define UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH 16

/* 
* Param stuff
*/
typedef struct
{
    uint8_t* name;
	uint8_t type;
    float val;
    float min;
    float max;
    float defval;
	uint8_t* pointer;
} param_t;

enum ap_var_type {
    AP_PARAM_NONE    = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
    AP_PARAM_VECTOR3F,
    AP_PARAM_GROUP
};

static param_t parameters[] =
{
	{"app_to_use", 		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
    {"vesc_id", 		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
    {"timeout", 		AP_PARAM_INT32, 0, 0, 0, 0, NULL},
    {"timeout_bk_curr",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"can_stat_mode", 	AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"can_stat_rate", 	AP_PARAM_INT32, 0, 0, 0, 0, NULL},
	{"can_baud_rate", 	AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"pairing_done", 	AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"en_perm_uart", 	AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"shutdown_mode", 	AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"can_mode", 		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"uavcan_index", 	AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"uavcan_raw_md", 	AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"motor_type", 			AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"inv_mot_dir", 		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"sensor_mode", 		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"abi_enc_cnt", 		AP_PARAM_INT32,	0, 0, 0, 0, NULL},
	{"l_curr_max",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_curr_min",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_in_curr_max",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_in_curr_min",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_abs_curr_max",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_min_erpm",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_max_erpm",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_erpm_start",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_max_erpm_fbk",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_max_erpm_fbkcc",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_min_vin",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_max_vin",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_batt_cut_start",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_batt_cut_end",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_slow_abs_curr",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"l_tmp_fet_start",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_tmp_fet_end",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_tmp_mot_start",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_tmp_mot_end",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_tmp_accel_dec",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_min_duty",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_max_duty",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_watt_max",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_watt_min",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_curr_max_scale",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_curr_min_scale",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"l_duty_start",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"pwm_mode",			AP_PARAM_INT8,  0, 0, 0, 0, NULL},
	{"comm_mode",			AP_PARAM_INT8,  0, 0, 0, 0, NULL},
	{"motor_type",			AP_PARAM_INT8,  0, 0, 0, 0, NULL},
	{"sensor_mode",			AP_PARAM_INT8,  0, 0, 0, 0, NULL},
	{"sl_min_erpm",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"sl_minerpm_int_l", 	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"sl_mx_fb_curdirc",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"sl_cycle_int_lim",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"sl_ph_adv_at_br",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"sl_cycint_rpm_br",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"sl_bemf_coup_k",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"hall_table_0",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_table_1",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_table_2",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_table_3",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_table_4",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_table_5",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_table_6",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_table_7",		AP_PARAM_INT8, 	0, 0, 0, 0, NULL},
	{"hall_sl_erpm",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_current_kp",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_current_ki",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_f_sw",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_dt_us",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_enc_offset",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_enc_inverted", 	AP_PARAM_INT8,  0, 0, 0, 0, NULL},
	{"foc_enc_ratio",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_enc_sin_offs",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_enc_sin_gain",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_enc_cos_offs",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_enc_cos_gain",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_encsincosflt",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_mot_l",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_mot_ld_lq_df",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_mot_r",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_mot_flux_lnk",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_obs_gain",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_obs_gain_sl",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_pll_kp",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_pll_ki",			AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_dty_dwrmp_kp",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_dty_dwrmp_ki",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_oplp_rpm",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_oplp_rpm_low",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_d_gn_scl_st",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_d_gn_scl_mxm",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_sl_oplp_hyst",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_sl_oplp_tm",		AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_sl_oplp_tm_l",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
	{"foc_sl_oplp_tm_r",	AP_PARAM_FLOAT, 0, 0, 0, 0, NULL},
};

static inline param_t* getParamByIndex(uint16_t index)
{
    if (index >= sizeof(parameters))
    {
        return NULL;
    }
    return &parameters[index];
}
 
 
static inline param_t* getParamByName(uint8_t * name)
{
    for (uint16_t i = 0; i < sizeof(parameters); i++)
    {
		if(name == parameters[i].name)
        // if (strncmp(name, parameters[i].name, strlen(parameters[i].name)) == 0)
        {
              return &parameters[i];
        }
    } 
    return NULL;
}

static inline void updateParamByName(uint8_t * name, float value) 
{
	param_t* p = NULL;
	p = getParamByName(name);
	if (p != NULL) {
		if(p->val != value) {
		commands_printf("%s, %s p->val %0.02f, value %0.02f", p->name, name, p->val, value);	
		p->val = value;
		}
	} else {
		commands_printf("UAVCAN updateParamByName(): Parameter name not found");
	}
}

/*
 * Node status variables
 */
static uavcan_protocol_NodeStatus node_status;

// Private datatypes
typedef struct {
	int id;
	systime_t rx_time;
	uavcan_equipment_esc_Status msg;
} status_msg_wrapper_t;

// Private variables
static CanardInstance canard;
static uint8_t canard_memory_pool[1024];
static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
static int debug_level;
static status_msg_wrapper_t stat_msgs[STATUS_MSGS_TO_STORE];

// Threads
static THD_WORKING_AREA(canard_thread_wa, 2048);
static THD_FUNCTION(canard_thread, arg);

// Private functions
static void sendEscStatus(void);
static void readUniqueID(uint8_t* out_uid);
static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE]);
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
static bool shouldAcceptTransfer(const CanardInstance* ins,
		uint64_t* out_data_type_signature,
		uint16_t data_type_id,
		CanardTransferType transfer_type,
		uint8_t source_node_id);
static void terminal_debug_on(int argc, const char **argv);

void canard_driver_init(void) {
	debug_level = 0;

	for (int i = 0;i < STATUS_MSGS_TO_STORE;i++) {
		stat_msgs[i].id = -1;
	}

	chThdCreateStatic(canard_thread_wa, sizeof(canard_thread_wa), NORMALPRIO, canard_thread, NULL);

	terminal_register_command_callback(
			"uavcan_debug",
			"Enable UAVCAN debug prints (0 = off)",
			"[level]",
			terminal_debug_on);
}

static void sendEscStatus(void) {
	uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
	uavcan_equipment_esc_Status status;
	status.current = mc_interface_get_tot_current();
	status.error_count = mc_interface_get_fault();
	status.esc_index = app_get_configuration()->uavcan_esc_index;
	status.power_rating_pct = (fabsf(mc_interface_get_tot_current()) /
			mc_interface_get_configuration()->l_current_max *
			mc_interface_get_configuration()->l_current_max_scale) * 100.0;
	status.rpm = mc_interface_get_rpm();
	status.temperature = mc_interface_temp_fet_filtered() + 273.15;
	status.voltage = GET_INPUT_VOLTAGE();

	uavcan_equipment_esc_Status_encode(&status, buffer);

	static uint8_t transfer_id;


	if (debug_level > 4) {
		commands_printf("UAVCAN sendESCStatus");
	}

	canardBroadcast(&canard,
			UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
			UAVCAN_EQUIPMENT_ESC_STATUS_ID,
			&transfer_id,
			CANARD_TRANSFER_PRIORITY_LOW,
			buffer,
			UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE);
}

static void readUniqueID(uint8_t* out_uid) {
	uint8_t len = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_UNIQUE_ID_MAX_LENGTH;
    memset(out_uid, 0, len);
    memcpy(out_uid, (const void *)STM32_UUID_8, UNIQUE_ID_LENGTH_BYTES);
}

static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE]) {
	memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
	const uint32_t uptime_sec = ST2S(chVTGetSystemTimeX());
	canardEncodeScalar(buffer,  0, 32, &uptime_sec);
	canardEncodeScalar(buffer, 32,  2, &node_health);
	canardEncodeScalar(buffer, 34,  3, &node_mode);
}

/*
  handle a GET_NODE_INFO request
*/
static void handle_get_node_info(CanardInstance* ins, CanardRxTransfer* transfer) {
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
    uavcan_protocol_GetNodeInfoResponse pkt;

    node_status.uptime_sec = ST2S(chVTGetSystemTimeX());

    pkt.status = node_status;
    pkt.software_version.major = FW_VERSION_MAJOR;
    pkt.software_version.minor = FW_VERSION_MINOR;
    pkt.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT | UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC;
    pkt.software_version.vcs_commit = 0;
    uint32_t *crc = (uint32_t *)&pkt.software_version.image_crc;
    crc[0] = 0;
    crc[1] = 0;

	readUniqueID(pkt.hardware_version.unique_id);

    // use hw major/minor for APJ_BOARD_ID so we know what fw is
    // compatible with this hardware
    pkt.hardware_version.major = HW_MAJOR;
    pkt.hardware_version.minor = HW_MINOR;

    char name[strlen(CAN_APP_NODE_NAME)+1];
    strcpy(name, CAN_APP_NODE_NAME);
    pkt.name.len = strlen(CAN_APP_NODE_NAME);
    pkt.name.data = (uint8_t *)name;

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    const int16_t resp_res = canardRequestOrRespond(ins,
                                                    transfer->source_node_id,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_ID,
                                                    &transfer->transfer_id,
                                                    transfer->priority,
                                                    CanardResponse,
                                                    &buffer[0],
                                                    total_size);
    if (resp_res <= 0) {
        commands_printf("Could not respond to GetNodeInfo: %d\n", resp_res);
    }
}

/*
  handle ESC Raw command
*/
static void handle_esc_raw_command(CanardInstance* ins, CanardRxTransfer* transfer) {
	uavcan_equipment_esc_RawCommand cmd;
	uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];
	memset(buffer, 0, sizeof(buffer));
	uint8_t *tmp = buffer;

	if (uavcan_equipment_esc_RawCommand_decode_internal(transfer, transfer->payload_len, &cmd, &tmp, 0) >= 0) {
		if (cmd.cmd.len > app_get_configuration()->uavcan_esc_index) {
			float raw_val = ((float)cmd.cmd.data[app_get_configuration()->uavcan_esc_index]) / 8192.0;

			switch (app_get_configuration()->uavcan_raw_mode) {
				case UAVCAN_RAW_MODE_CURRENT:
					mc_interface_set_current_rel(raw_val);
					break;

				case UAVCAN_RAW_MODE_CURRENT_NO_REV_BRAKE:
					if (raw_val >= 0.0) {
						mc_interface_set_current_rel(raw_val);
					} else {
						mc_interface_set_brake_current_rel(-raw_val);
					}
					break;

				case UAVCAN_RAW_MODE_DUTY:
					mc_interface_set_duty(raw_val);
					break;

				default:
					break;
			}
			timeout_reset();
		}
	}
}

/*
  handle ESC RPM command
*/
static void handle_esc_rpm_command(CanardInstance* ins, CanardRxTransfer* transfer) {
	uavcan_equipment_esc_RPMCommand cmd;
	uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_MAX_SIZE];
	memset(buffer, 0, sizeof(buffer));
	uint8_t *tmp = buffer;

	if (uavcan_equipment_esc_RPMCommand_decode_internal(transfer, transfer->payload_len, &cmd, &tmp, 0) >= 0) {
		if (cmd.rpm.len > app_get_configuration()->uavcan_esc_index) {
			mc_interface_set_pid_speed(cmd.rpm.data[app_get_configuration()->uavcan_esc_index]);
			timeout_reset();
		}
	}
}

/*
  handle Equipment ESC Status Request
*/
static void handle_esc_status_req(CanardInstance* ins, CanardRxTransfer* transfer) {
	uavcan_equipment_esc_Status msg;
	if (uavcan_equipment_esc_Status_decode_internal(transfer, transfer->payload_len, &msg, 0, 0) >= 0) {
		for (int i = 0;i < STATUS_MSGS_TO_STORE;i++) {
			status_msg_wrapper_t *msgw = &stat_msgs[i];
			if (msgw->id == -1 || msgw->id == transfer->source_node_id) {
				msgw->id = transfer->source_node_id;
				msgw->rx_time = chVTGetSystemTimeX();
				msgw->msg = msg;
				break;
			}
		}
	}
}

static void update_params() 
{
	const app_configuration *conf = app_get_configuration();
	updateParamByName((uint8_t *)"app_to_use", 			conf->app_to_use);
	updateParamByName((uint8_t *)"vesc_id", 			conf->controller_id);
	updateParamByName((uint8_t *)"timeout", 			conf->timeout_msec );	
	updateParamByName((uint8_t *)"timeout_bk_curr", 	conf->timeout_brake_current );	
	updateParamByName((uint8_t *)"can_stat_mode", 		conf->send_can_status );	
	updateParamByName((uint8_t *)"can_stat_rate", 		conf->send_can_status_rate_hz );	
	updateParamByName((uint8_t *)"can_baud_rate", 		conf->can_baud_rate );	
	updateParamByName((uint8_t *)"pairing_done", 		conf->pairing_done);	
	updateParamByName((uint8_t *)"en_perm_uart", 		conf->permanent_uart_enabled );	
	updateParamByName((uint8_t *)"shutdown_mode", 		conf->shutdown_mode );	
	updateParamByName((uint8_t *)"can_mode", 			conf->can_mode );	
	updateParamByName((uint8_t *)"uavcan_index", 		conf->uavcan_esc_index );	
	updateParamByName((uint8_t *)"uavcan_raw_md", 		conf->uavcan_raw_mode );	

	mc_configuration *mcconf = mc_interface_get_configuration();
	// Motor Conf General Tab
	updateParamByName((uint8_t *)"motor_type", 			mcconf->motor_type );
	updateParamByName((uint8_t *)"inv_mot_dir", 		mcconf->m_invert_direction );
	updateParamByName((uint8_t *)"sensor_mode", 		mcconf->sensor_mode );
	updateParamByName((uint8_t *)"abi_enc_cnt", 		mcconf->m_encoder_counts );

	updateParamByName((uint8_t *)"l_curr_max",			mcconf->l_current_max		);
	updateParamByName((uint8_t *)"l_curr_min",			mcconf->l_current_min		);
	updateParamByName((uint8_t *)"l_in_curr_max",		mcconf->l_in_current_max	);
	updateParamByName((uint8_t *)"l_in_curr_min",		mcconf->l_in_current_min	);
	updateParamByName((uint8_t *)"l_abs_curr_max",		mcconf->l_abs_current_max	);
	updateParamByName((uint8_t *)"l_min_erpm",			mcconf->l_min_erpm			);
	updateParamByName((uint8_t *)"l_max_erpm",			mcconf->l_max_erpm			);
	updateParamByName((uint8_t *)"l_erpm_start",		mcconf->l_erpm_start		);
	updateParamByName((uint8_t *)"l_max_erpm_fbk",		mcconf->l_max_erpm_fbrake	);
	updateParamByName((uint8_t *)"l_max_erpm_fbkcc",	mcconf->l_max_erpm_fbrake_cc);
	updateParamByName((uint8_t *)"l_min_vin",			mcconf->l_min_vin			);
	updateParamByName((uint8_t *)"l_max_vin",			mcconf->l_max_vin			);
	updateParamByName((uint8_t *)"l_batt_cut_start",	mcconf->l_battery_cut_start );
	updateParamByName((uint8_t *)"l_batt_cut_end",		mcconf->l_battery_cut_end	);
	updateParamByName((uint8_t *)"l_slow_abs_curr",		mcconf->l_slow_abs_current	);
	updateParamByName((uint8_t *)"l_tmp_fet_start",		mcconf->l_temp_fet_start	);
	updateParamByName((uint8_t *)"l_tmp_fet_end",		mcconf->l_temp_fet_end		);
	updateParamByName((uint8_t *)"l_tmp_mot_start",		mcconf->l_temp_motor_start	);
	updateParamByName((uint8_t *)"l_tmp_mot_end",		mcconf->l_temp_motor_end	);
	updateParamByName((uint8_t *)"l_tmp_accel_dec",		mcconf->l_temp_accel_dec	);
	updateParamByName((uint8_t *)"l_min_duty",			mcconf->l_min_duty			);
	updateParamByName((uint8_t *)"l_max_duty",			mcconf->l_max_duty			);
	updateParamByName((uint8_t *)"l_watt_max",			mcconf->l_watt_max			);
	updateParamByName((uint8_t *)"l_watt_min",			mcconf->l_watt_min			);
	updateParamByName((uint8_t *)"l_curr_max_scale",	mcconf->l_current_max_scale );
	updateParamByName((uint8_t *)"l_curr_min_scale",	mcconf->l_current_min_scale );
	updateParamByName((uint8_t *)"l_duty_start",		mcconf->l_duty_start		);
	updateParamByName((uint8_t *)"pwm_mode", 			mcconf->pwm_mode);
	updateParamByName((uint8_t *)"comm_mode",			mcconf->comm_mode);
	updateParamByName((uint8_t *)"motor_type",			mcconf->motor_type);
	updateParamByName((uint8_t *)"sensor_mode",			mcconf->sensor_mode);
	updateParamByName((uint8_t *)"sl_min_erpm",			mcconf->sl_min_erpm);
	updateParamByName((uint8_t *)"sl_minerpm_int_l",	mcconf->sl_min_erpm_cycle_int_limit);
	updateParamByName((uint8_t *)"sl_mx_fb_curdirc",	mcconf->sl_max_fullbreak_current_dir_change);
	updateParamByName((uint8_t *)"sl_cycle_int_lim",	mcconf->sl_cycle_int_limit);
	updateParamByName((uint8_t *)"sl_ph_adv_at_br",		mcconf->sl_phase_advance_at_br);
	updateParamByName((uint8_t *)"sl_cycint_rpm_br",	mcconf->sl_cycle_int_rpm_br);
	updateParamByName((uint8_t *)"sl_bemf_coup_k",		mcconf->sl_bemf_coupling_k);
	updateParamByName((uint8_t *)"hall_table_0",		mcconf->hall_table[0]);
	updateParamByName((uint8_t *)"hall_table_1",		mcconf->hall_table[1]);
	updateParamByName((uint8_t *)"hall_table_2",		mcconf->hall_table[2]);
	updateParamByName((uint8_t *)"hall_table_3",		mcconf->hall_table[3]);
	updateParamByName((uint8_t *)"hall_table_4",		mcconf->hall_table[4]);
	updateParamByName((uint8_t *)"hall_table_5",		mcconf->hall_table[5]);
	updateParamByName((uint8_t *)"hall_table_6",		mcconf->hall_table[6]);
	updateParamByName((uint8_t *)"hall_table_7",		mcconf->hall_table[7]);
	updateParamByName((uint8_t *)"hall_sl_erpm",		mcconf->hall_sl_erpm);
	updateParamByName((uint8_t *)"foc_current_kp",		mcconf->foc_current_kp);
	updateParamByName((uint8_t *)"foc_current_ki",		mcconf->foc_current_ki);
	updateParamByName((uint8_t *)"foc_f_sw",			mcconf->foc_f_sw);
	updateParamByName((uint8_t *)"foc_dt_us",			mcconf->foc_dt_us);
	updateParamByName((uint8_t *)"foc_enc_offset",		mcconf->foc_encoder_offset);
	updateParamByName((uint8_t *)"foc_enc_inverted",	mcconf->foc_encoder_inverted);
	updateParamByName((uint8_t *)"foc_enc_ratio",		mcconf->foc_encoder_ratio);
	updateParamByName((uint8_t *)"foc_enc_sin_offs",	mcconf->foc_encoder_sin_offset);
	updateParamByName((uint8_t *)"foc_enc_sin_gain",	mcconf->foc_encoder_sin_gain);
	updateParamByName((uint8_t *)"foc_enc_cos_offs",	mcconf->foc_encoder_cos_offset);
	updateParamByName((uint8_t *)"foc_enc_cos_gain",	mcconf->foc_encoder_cos_gain);
	updateParamByName((uint8_t *)"foc_encsincosflt",	mcconf->foc_encoder_sincos_filter_constant);
	updateParamByName((uint8_t *)"foc_mot_l",			mcconf->foc_motor_l);
	updateParamByName((uint8_t *)"foc_mot_ld_lq_df",	mcconf->foc_motor_ld_lq_diff);
	updateParamByName((uint8_t *)"foc_mot_r",			mcconf->foc_motor_r);
	updateParamByName((uint8_t *)"foc_mot_flux_lnk",	mcconf->foc_motor_flux_linkage);
	updateParamByName((uint8_t *)"foc_obs_gain",		mcconf->foc_observer_gain);
	updateParamByName((uint8_t *)"foc_obs_gain_sl",		mcconf->foc_observer_gain_slow);
	updateParamByName((uint8_t *)"foc_pll_kp",			mcconf->foc_pll_kp);
	updateParamByName((uint8_t *)"foc_pll_ki",			mcconf->foc_pll_ki);
	updateParamByName((uint8_t *)"foc_dty_dwrmp_kp",	mcconf->foc_duty_dowmramp_kp);
	updateParamByName((uint8_t *)"foc_dty_dwrmp_ki",	mcconf->foc_duty_dowmramp_ki);
	updateParamByName((uint8_t *)"foc_oplp_rpm",		mcconf->foc_openloop_rpm);
	updateParamByName((uint8_t *)"foc_oplp_rpm_low",	mcconf->foc_openloop_rpm_low);
	updateParamByName((uint8_t *)"foc_d_gn_scl_st",		mcconf->foc_d_gain_scale_start);
	updateParamByName((uint8_t *)"foc_d_gn_scl_mxm",	mcconf->foc_d_gain_scale_max_mod);
	updateParamByName((uint8_t *)"foc_sl_oplp_hyst",	mcconf->foc_sl_openloop_hyst);
	updateParamByName((uint8_t *)"foc_sl_oplp_tm",		mcconf->foc_sl_openloop_time);
	updateParamByName((uint8_t *)"foc_sl_oplp_tm_l",	mcconf->foc_sl_openloop_time_lock);
	updateParamByName((uint8_t *)"foc_sl_oplp_tm_r",	mcconf->foc_sl_openloop_time_ramp);

	// mempools_free_mcconf(mcconf);
}

/*
  handle parameter GetSet request
*/
static void handle_param_getset(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_protocol_param_GetSetRequest req;
    uint8_t arraybuf[UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_NAME_MAX_LENGTH];
    uint8_t *arraybuf_ptr = arraybuf;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, transfer->payload_len, &req, &arraybuf_ptr) < 0) {
        return;
    }

	update_params();

	uavcan_protocol_param_GetSetResponse pkt;

	uint8_t name[AP_MAX_NAME_SIZE+1] = "";

	param_t* p = NULL;

    if (req.name.len != 0 && req.name.len > AP_MAX_NAME_SIZE) {
		commands_printf("UAVCAN param_getset: Parameter Name is too long!");
        p = NULL;
    } else if (req.name.len != 0 && req.name.len <= AP_MAX_NAME_SIZE) {
        strncpy((char *)name, (char *)req.name.data, req.name.len);
		commands_printf("UAVCAN param_getset param name: %s", name);
		p = getParamByName(name);
    } else {
		commands_printf("UAVCAN param_getset param index: %d", req.index);
		p = getParamByIndex(req.index);
    }

	if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
		// Set request and valid parameter found
		switch (req.value.union_tag) {
			case UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY:
				return;
			break;

			case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
				p->val = req.value.integer_value;
			break;
		}
	}

	if(p != NULL) {
		uint8_t arrSize = strlen(p->name);
		// strncpy((char *)name, (char *)p->name, arrSize);
		// commands_printf("UAVCAN param_getset got param name: %s size: %d", (char *)p->name, arrSize);
		// commands_printf("UAVCAN param_getset: value: %0.02f", p->val);

		switch(p->type) {
			case AP_PARAM_INT8:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.value.integer_value = (int8_t)p->val;
				pkt.default_value.integer_value = (int8_t)p->defval;
				pkt.min_value.integer_value = (int8_t)p->min;
				pkt.max_value.integer_value = (int8_t)p->max;
			break;

			case AP_PARAM_INT16:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.value.integer_value = (int16_t)p->val;
				pkt.default_value.integer_value = (int16_t)p->defval;
				pkt.min_value.integer_value = (int16_t)p->min;
				pkt.max_value.integer_value = (int16_t)p->max;
			break;

			case AP_PARAM_INT32:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
				pkt.value.integer_value = (int32_t)p->val;
				pkt.default_value.integer_value = (int32_t)p->defval;
				pkt.min_value.integer_value = (int32_t)p->min;
				pkt.max_value.integer_value = (int32_t)p->max;
			break;

			case AP_PARAM_FLOAT:
				pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
				pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
				pkt.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
				pkt.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
				pkt.value.real_value = (float)p->val;
				pkt.default_value.real_value = (float)p->defval;
				pkt.min_value.real_value = (float)p->min;
				pkt.max_value.real_value = (float)p->max;
			break;
		}
		
		pkt.name.len = strlen(p->name);
		pkt.name.data = (char *)p->name;
		
	}

	uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
	uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

	const int16_t resp_res = canardRequestOrRespond(ins,
													transfer->source_node_id,
													UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
													UAVCAN_PROTOCOL_PARAM_GETSET_ID,
													&transfer->transfer_id,
													transfer->priority,
													CanardResponse,
													&buffer[0],
													total_size);
	if (resp_res <= 0) {
        commands_printf("Could not respond to param_getset_req: %d\n", resp_res);
    }												
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer) {
	if (debug_level > 3) {
		commands_printf("UAVCAN transfer RX: NODE: %d Type: %d ID: %d",
				transfer->source_node_id, transfer->transfer_type, transfer->data_type_id);
	}

    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    // if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) {
    //     if (transfer->transfer_type == CanardTransferTypeBroadcast &&
    //         transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
    //         handle_allocation_response(ins, transfer);
    //     }
    //     return;
    // }

   	switch (transfer->data_type_id) {
		case UAVCAN_PROTOCOL_GETNODEINFO_ID:
			handle_get_node_info(ins, transfer);
			break;

		case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
			handle_esc_raw_command(ins, transfer);
			break;

		case UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID:
			handle_esc_rpm_command(ins, transfer);
			break;

		case UAVCAN_EQUIPMENT_ESC_STATUS_ID:
			handle_esc_status_req(ins, transfer);
			break;

		case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
			handle_param_getset(ins, transfer);
			break;
   	}
}

/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)source_node_id;

	if (debug_level > 3) {
		commands_printf("UAVCAN shouldAccept: NODE: %d Type: %d ID: %d",
				source_node_id, transfer_type, data_type_id);
	}

	// This is for future use if Dynamic node ID allocation is used.
    // if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    // {
    //     /*
    //      * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
    //      */
    //     if ((transfer_type == CanardTransferTypeBroadcast) &&
    //         (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID))
    //     {
    //         *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
    //         return true;
    //     }
    //     return false;
    // }

    switch (data_type_id) {
		case UAVCAN_GET_NODE_INFO_DATA_TYPE_ID:
			*out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
			return true;

		case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID:
			*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
			return true;

		case UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID:
			*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_SIGNATURE;
			return true;

		case UAVCAN_EQUIPMENT_ESC_STATUS_ID:
			*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE;
			return true;

		case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
			*out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
			return true;

		default:
			break;
    }

    return false;
}

static void terminal_debug_on(int argc, const char **argv) {
	if (argc == 2) {
		int level = -1;
		sscanf(argv[1], "%d", &level);

		if (level >= 0) {
			debug_level = level;
			commands_printf("UAVCAN debug level is now %d", debug_level);
		} else {
			commands_printf("Invalid argument(s).\n");
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static THD_FUNCTION(canard_thread, arg) {
	(void)arg;
	chRegSetThreadName("UAVCAN");

	canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);

	systime_t last_status_time = 0;
	systime_t last_esc_status_time = 0;

	update_params();

	for (;;) {
		const app_configuration *conf = app_get_configuration();

		if (conf->can_mode != CAN_MODE_UAVCAN) {
			chThdSleepMilliseconds(100);
			continue;
		}

		
		canardSetLocalNodeID(&canard, conf->controller_id);

		CANRxFrame *rxmsg;
		while ((rxmsg = comm_can_get_rx_frame()) != 0) {
			CanardCANFrame rx_frame;

			if (rxmsg->IDE == CAN_IDE_EXT) {
				rx_frame.id = rxmsg->EID | CANARD_CAN_FRAME_EFF;
			} else {
				rx_frame.id = rxmsg->SID;
			}

			rx_frame.data_len = rxmsg->DLC;
			memcpy(rx_frame.data, rxmsg->data8, rxmsg->DLC);

			canardHandleRxFrame(&canard, &rx_frame, ST2US(chVTGetSystemTimeX()));
		}

		for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
			comm_can_transmit_eid(txf->id, txf->data, txf->data_len);
			canardPopTxQueue(&canard);
		}

		if (ST2MS(chVTTimeElapsedSinceX(last_status_time)) >= 1000) {
			last_status_time = chVTGetSystemTimeX();
			canardCleanupStaleTransfers(&canard, ST2US(chVTGetSystemTimeX()));

			uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
			makeNodeStatusMessage(buffer);

			static uint8_t transfer_id;
			canardBroadcast(&canard,
					UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
					UAVCAN_NODE_STATUS_DATA_TYPE_ID,
					&transfer_id,
					CANARD_TRANSFER_PRIORITY_LOW,
					buffer,
					UAVCAN_NODE_STATUS_MESSAGE_SIZE);
		}

		if (ST2MS(chVTTimeElapsedSinceX(last_esc_status_time)) >= 1000 / conf->send_can_status_rate_hz &&
				conf->send_can_status != CAN_STATUS_DISABLED) {
			last_esc_status_time = chVTGetSystemTimeX();
			sendEscStatus();
		}

		chThdSleepMilliseconds(1);
	}
}
