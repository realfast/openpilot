const uint32_t CHRYSLER_RT_INTERVAL = 250000;  // 250ms between real time checks
const int CHRYSLER_RAM_STANDSTILL_THRSLD = 3;  // about 1m/s changed from wheel rpm to km/h

const int CHRYSLER_RAM_HD_MAX_STEER = 361;
const int CHRYSLER_RAM_HD_MAX_RATE_UP = 14;
const int CHRYSLER_RAM_HD_MAX_RATE_DOWN = 14;
const int CHRYSLER_RAM_HD_MAX_TORQUE_ERROR = 400; 
const int CHRYSLER_RAM_HD_MAX_RT_DELTA = 182;

// CAN messages for Chrysler/Jeep platforms
#define EPS_2                      544  // EPS driver input torque
#define ESP_1                      320  // Brake pedal and vehicle speed
#define ESP_8                      284  // Brake pedal and vehicle speed
#define ECM_5                      559  // Throttle position sensor
#define DAS_3                      500  // ACC engagement states from DASM

// CAN messages for the 5th gen RAM HD platform
#define DAS_6_HD                   629  // LKAS HUD and auto headlight control from DASM
#define LKAS_COMMAND_HD            630  // LKAS controls from DASM 
#define CRUISE_BUTTONS_HD          570  // Cruise control buttons
#define Center_Stack_2_RAM         650  // Center Stack buttons

const CanMsg CHRYSLER_RAM_HD_TX_MSGS[] = {
  {CRUISE_BUTTONS_HD, 2, 3},
  {LKAS_COMMAND_HD, 0, 8},
  {LKAS_COMMAND_HD, 1, 8}, 
  {DAS_6_HD, 0, 8},
  {DAS_6_HD, 1, 8},
  {ESP_8, 1, 8},
  {DAS_3, 0, 8},
  {DAS_3, 1, 8},
};

AddrCheckStruct chrysler_ram_hd_addr_checks[] = {
  {.msg = {{EPS_2, 1, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 10000U}, { 0 }, { 0 }}},  // EPS module
  {.msg = {{ESP_1, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},  // brake pressed
  {.msg = {{ESP_8, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},  // vehicle Speed
  {.msg = {{ECM_5, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},  // gas pedal
  {.msg = {{DAS_3, 2, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},  // cruise state
};
#define CHRYSLER_RAM_HD_ADDR_CHECK_LEN (sizeof(chrysler_ram_hd_addr_checks) / sizeof(chrysler_ram_hd_addr_checks[0]))

addr_checks chrysler_rx_checks  = {chrysler_ram_hd_addr_checks, CHRYSLER_RAM_HD_ADDR_CHECK_LEN};

static uint32_t chrysler_get_checksum(CANPacket_t *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1U;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte));
}

static uint32_t chrysler_compute_checksum(CANPacket_t *to_push) {
  // TODO: clean this up
  // http://illmatics.com/Remote%20Car%20Hacking.pdf
  uint8_t checksum = 0xFFU;
  int len = GET_LEN(to_push);
  for (int j = 0; j < (len - 1); j++) {
    uint8_t shift = 0x80U;
    uint8_t curr = (uint8_t)GET_BYTE(to_push, j);
    for (int i=0; i<8; i++) {
      uint8_t bit_sum = curr & shift;
      uint8_t temp_chk = checksum & 0x80U;
      if (bit_sum != 0U) {
        bit_sum = 0x1C;
        if (temp_chk != 0U) {
          bit_sum = 1;
        }
        checksum = checksum << 1;
        temp_chk = checksum | 1U;
        bit_sum ^= temp_chk;
      } else {
        if (temp_chk != 0U) {
          bit_sum = 0x1D;
        }
        checksum = checksum << 1;
        bit_sum ^= checksum;
      }
      checksum = bit_sum;
      shift = shift >> 1;
    }
  }
  return (uint8_t)(~checksum);
}

static uint8_t chrysler_get_counter(CANPacket_t *to_push) {
  return (uint8_t)(GET_BYTE(to_push, 6) >> 4);
}

static int chrysler_rx_hook(CANPacket_t *to_push) {

  bool valid = addr_safety_check(to_push, &chrysler_rx_checks,
                                 chrysler_get_checksum, chrysler_compute_checksum,
                                 chrysler_get_counter);

  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if (valid) {

    // Measured EPS torque
    const int eps_2 = EPS_2;
    if ((bus == 1) && (addr == eps_2)) {
      int torque_meas_new = ((GET_BYTE(to_push, 4) & 0x7U) << 8) + GET_BYTE(to_push, 5) - 1024U;
      update_sample(&torque_meas, torque_meas_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    const int das_3 = DAS_3;
    const int das_3_bus = 2;
    if ((bus == das_3_bus) && (addr == das_3)) {
      int cruise_engaged = GET_BIT(to_push, 21U) == 1U;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // TODO: use the same message for both
    // update speed
    if (addr == ESP_8) {
      vehicle_speed = (((GET_BYTE(to_push, 4) & 0x3U) << 8) + GET_BYTE(to_push, 5))*0.0078125;
      vehicle_moving = (int)vehicle_speed > CHRYSLER_RAM_STANDSTILL_THRSLD;
    }

    // exit controls on rising edge of gas press
    const int ecm_5 = ECM_5;
    if ((bus == 0) && (addr == ecm_5)) {
      gas_pressed = GET_BYTE(to_push, 0U) != 0U;
    }

    // exit controls on rising edge of brake press
    const int esp_1 = ESP_1;
    if ((bus == 0) && (addr == esp_1)) {
      brake_pressed = ((GET_BYTE(to_push, 0U) & 0xFU) >> 2U) == 1U;
    }

    const int lkas_command = LKAS_COMMAND_HD;
    generic_rx_checks((bus == 0) && (addr == lkas_command));
  }
  return valid;
}

static int chrysler_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {
  UNUSED(longitudinal_allowed);

  int tx = 1;
  int addr = GET_ADDR(to_send);

  tx = msg_allowed(to_send, CHRYSLER_RAM_HD_TX_MSGS, sizeof(CHRYSLER_RAM_HD_TX_MSGS) / sizeof(CHRYSLER_RAM_HD_TX_MSGS[0]));

  // STEERING
  const int lkas_addr =  LKAS_COMMAND_HD;
  if (tx && (addr == lkas_addr)) {
    int start_byte = 1;
    int desired_torque = ((GET_BYTE(to_send, start_byte) & 0x7U) << 8) | GET_BYTE(to_send, start_byte + 1);
    desired_torque -= 1024;

    uint32_t ts = microsecond_timer_get();
    bool violation = 0;

    if (controls_allowed) {
      // *** global torque limit check ***
      const int max_rt_delta = CHRYSLER_RAM_HD_MAX_RT_DELTA;
      const int max_error = CHRYSLER_RAM_HD_MAX_TORQUE_ERROR;
      
      violation |= max_limit_check(desired_torque, CHRYSLER_RAM_HD_MAX_STEER, -CHRYSLER_RAM_HD_MAX_STEER);

      // *** torque rate limit check ***
      const int max_rate_up = CHRYSLER_RAM_HD_MAX_RATE_UP;
      const int max_rate_down = CHRYSLER_RAM_HD_MAX_RATE_DOWN;
      violation |= dist_to_meas_check(desired_torque, desired_torque_last,
        &torque_meas, max_rate_up, max_rate_down, max_error);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, max_rt_delta);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > CHRYSLER_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  // if ((addr == CRUISE_BUTTONS)) {
  //   const bool is_cancel = GET_BYTE(to_send, 0) == 1U;
  //   const bool is_resume = GET_BYTE(to_send, 0) == 0x10U;
  //   const bool allowed = is_cancel || (is_resume && controls_allowed);
  //   if (!allowed) {
  //     tx = 0;
  //   }
  // }

  return tx;
}

static int chrysler_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  // forward CAN 0 & 1 -> 2 so stock LKAS camera sees messages
  if (bus_num == 0){
    if ((addr == ESP_8)) {
      bus_fwd = 2;
    }
    else if ((addr == CRUISE_BUTTONS_HD) || (addr == Center_Stack_2_RAM)){
      bus_fwd = 1;
    }
    else {//Ram and HD share the same
      //When forwarding to multiple addresses, make sure to use a hex value of the highest bus first (0xF0 spot/bits 4-7) and lowest bus second (0x0F spot/bits 0-3)
      //Bus 0 will be ignored if put in the high 4 bits
      bus_fwd = 0x21; //Sends to bus 2 and bus 1
    }
  }

  // forward all messages from camera except LKAS messages
  const bool is_lkas = ((addr == LKAS_COMMAND_HD) || (addr == DAS_6_HD) || (addr == DAS_3));
  if ((bus_num == 2) && !is_lkas){
      //When forwarding to multiple addresses, make sure to use a hex value of the highest bus first (0xF0 spot/bits 4-7) and lowest bus second (0x0F spot/bits 0-3)
      //Bus 0 will be ignored if put in the high 4 bits
      bus_fwd = 0x10;//Sends to bus 1 and bus 0
  }

  //forward CAN1->CAN2
  if (bus_num == 1){
    //When forwarding to multiple addresses, make sure to use a hex value of the highest bus first (0xF0 spot/bits 4-7) and lowest bus second (0x0F spot/bits 0-3)
    //Bus 0 will be ignored if put in the high 4 bits
    bus_fwd = 0x20;//Sends to bus 2 and bus 0
  }

  return bus_fwd;
}

static const addr_checks* chrysler_init(uint16_t param) {
  UNUSED(param);
  chrysler_rx_checks = (addr_checks){chrysler_ram_hd_addr_checks, CHRYSLER_RAM_HD_ADDR_CHECK_LEN};

  return &chrysler_rx_checks;
}

const safety_hooks chrysler_hooks = {
  .init = chrysler_init,
  .rx = chrysler_rx_hook,
  .tx = chrysler_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = chrysler_fwd_hook,
};