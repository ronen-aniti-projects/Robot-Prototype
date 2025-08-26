def drive_line(mode, target_distance_cm, cfg, left_motor_pwm, right_motor_pwm):
    
    set_logic(mode, cfg)
    
    ticks_per_rev = cfg["ticks_per_rev"]
    wheel_diameter_cm = cfg["wheel_diameter_m"] * 100.0
    wheel_circumference_cm =  np.pi * wheel_diameter_cm
    target_ticks = target_distance_cm / wheel_circumference_cm * ticks_per_rev
    
    prev_state_left = GPIO.input(cfg["left_encoder_pin"])
    prev_state_right = GPIO.input(cfg["right_encoder_pin"])

    count_left = 0
    count_right = 0
    
    base_duty = 50
    kp = 10
      
    error = 0 # left_ticks - right ticks

    while count_left < target_ticks or count_right < target_ticks:
        current_state_left = GPIO.input(cfg["left_encoder_pin"])
        current_state_right = GPIO.input(cfg["right_encoder_pin"])
        if (current_state_left != prev_state_left):
            count_left += 1
            prev_state_left = current_state_left
        if (current_state_right != prev_state_right):
            count_right += 1
            prev_state_right = current_state_right
    
        error = count_left - count_right

        adjustment = kp * error
    
        left_motor_pwm.ChangeDutyCycle(np.clip(base_duty - adjustment, 0, 100))
        right_motor_pwm.ChangeDutyCycle(np.clip(base_duty + adjustment, 0, 100))
        
        time.sleep(0.001)
    
    left_motor_pwm.ChangeDutyCycle(0)
    right_motor_pwm.ChangeDutyCycle(0)