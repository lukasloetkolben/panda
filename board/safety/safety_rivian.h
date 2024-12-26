const SteeringLimits RIVIAN_STEERING_LIMITS = {
  .max_steer = 350,
  .max_rt_delta = 300,           // 8 max rate up * 100Hz send rate * 250000 RT interval / 1000000 = 200 ; 200 * 1.5 for safety pad = 300
  .max_rt_interval = 250000,     // 250ms between real time checks
  .max_rate_up = 8,
  .max_rate_down = 8,
  .driver_torque_allowance = 15,
  .driver_torque_factor = 1,
  .type = TorqueDriverLimited,
};


const LongitudinalLimits RIVIAN_LONG_LIMITS = {
  .max_accel = 200,       // 2. m/s^2
  .min_accel = -350,       // -3.50 m/s^2
  .inactive_accel = 0,
};

const int FLAG_RIVIAN_LONG_CONTROL = 1;

const CanMsg RIVIAN_TX_MSGS[] = {
  {0x120, 0, 8}, // ACM_lkaHbaCmd
  {0x160, 0, 5}, // ACM_longitudinalRequest
};

RxCheck rivian_rx_checks[] = {
  {.msg = {{0x390, 0, 7, .frequency = 100U}, { 0 }, { 0 }}},  // EPAS_AdasStatus (steering angle)
  {.msg = {{0x38b, 0, 6, .frequency = 50U}, { 0 }, { 0 }}},   // ESPiB1 (speed)
  {.msg = {{0x150, 0, 7, .frequency = 50U}, { 0 }, { 0 }}},   // VDM_PropStatus (gas pedal)
  {.msg = {{0x38f, 0, 6, .frequency = 50U}, { 0 }, { 0 }}},   // iBESP2 (brakes)
  {.msg = {{0x162, 0, 8, .frequency = 100U}, { 0 }, { 0 }}},  // VDM_AdasSts
  {.msg = {{0x100, 2, 8, .frequency = 100U}, { 0 }, { 0 }}},  // ACM_Status (cruise state)
};

bool rivian_longitudinal = false;

static void rivian_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if(bus == 0) {
    // Driver torque
    if (addr == 0x380) {
      int torque_driver_new = ((((GET_BYTE(to_push, 2) << 4) | (GET_BYTE(to_push, 3) >> 4)) * 0.1) - 205);
      update_sample(&torque_driver, torque_driver_new);
    }

    // Steering angle
    if(addr == 0x390) {
      // Angle: (0.1 * val) - 819.2 in deg.
      // Store it 1/10 deg to match steering request
      int angle_meas_new = ((GET_BYTE(to_push, 5) << 6) | (GET_BYTE(to_push, 6) >> 2)) - 8192U;
      update_sample(&angle_meas, angle_meas_new);
    }

    // Vehicle speed
    if(addr == 0x38b){
      float speed = (GET_BYTE(to_push, 2)  * 0.4);
      vehicle_moving = ABS(speed) > 0.1;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // Gas pressed
    if(addr == 0x150){
      gas_pressed = (((GET_BYTE(to_push, 3) << 2) | (GET_BYTE(to_push, 4) >> 6)) != 0U);
    }

    // Brake pressed
    if(addr == 0x38f){
      brake_pressed = GET_BIT(to_push, 23U);
    }
  }

  if (bus == 2) {
    // Cruise state
    if(addr == 0x100) {
      bool cruise_engaged = (((GET_BYTE(to_push, 2)) >> 5) != 0U);
      pcm_cruise_check(cruise_engaged);
    }
  }

}


static bool rivian_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);

  // Steering control
  if (addr == 0x120) {
    int desired_torque = ((GET_BYTE(to_send, 2) << 3U) | (GET_BYTE(to_send, 3) >> 5U)) - 1024U;
    bool steer_req = GET_BIT(to_send, 28U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, RIVIAN_STEERING_LIMITS)) {
       tx = true;
    }
  }

  // Longitudinal control
  if(addr == 0x160) {
    if (rivian_longitudinal) {
      int raw_accel = ((GET_BYTE(to_send, 2) << 3) | (GET_BYTE(to_send, 3) >> 5)) - 1024U;
      if (longitudinal_accel_checks(raw_accel, RIVIAN_LONG_LIMITS)) {
        tx = false;
      }
    } else {
       tx = false;
    }
  }

  return tx;
}

static int rivian_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  bool block_msg = false;

  if(bus_num == 0) {
    if(!block_msg) {
      bus_fwd = 2;
    }
  }

  if(bus_num == 2) {
    // ACM_lkaHbaCmd
    if (addr == 0x120) {
      block_msg = true;
    }

    // ACM_longitudinalRequest
    if (rivian_longitudinal && (addr == 0x160)) {
      block_msg = true;
    }

    if(!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config rivian_init(uint16_t param) {
  rivian_longitudinal = GET_FLAG(param, FLAG_RIVIAN_LONG_CONTROL);

  safety_config ret;
  ret = BUILD_SAFETY_CFG(rivian_rx_checks, RIVIAN_TX_MSGS);
  return ret;
}

const safety_hooks rivian_hooks = {
  .init = rivian_init,
  .rx = rivian_rx_hook,
  .tx = rivian_tx_hook,
  .fwd = rivian_fwd_hook,
};
