use ctrlc;
use runtime::hal::{Servo, ServoRegister, ServoMode, ServoDirection};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;
use std::env;
use anyhow::{Result, bail};
use clap::Parser;
use std::fs::OpenOptions;
use std::io::Write;


// TorqueLimit 10 = 1 % 100 = 10 % 

const MIN_SPEED: u16 = 10;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    servo_id: u8,
    
    #[arg(short, long, default_value_t = 150)]
    calibration_speed: u16,
    
    #[arg(short, long, default_value_t = 200.0)]
    current_threshold: f32,
}

#[derive(Debug)]
struct CalibrationRecord {
    servo_id: u8,
    iteration: u32,
    min_angle: i32,
    max_angle: i32,
    min_position: i32,
    max_position: i32,
}

pub fn auto_calibrate_servo(servo: &Servo, servo_id: u8, running: &Arc<AtomicBool>, calibration_speed: u16, current_threshold: f32) -> Result<()> {
    println!("Starting auto calibration for servo ID: {}", servo_id);
    println!("Calibration speed: {}", calibration_speed);
    println!("Current threshold: {} mA", current_threshold);

    servo.disable_readout()?;
    servo.set_mode(servo_id, ServoMode::ConstantSpeed)?; // Set to continuous mode

    let mut min_angle = 0;
    let mut max_angle = 0;
    let mut min_position = 0;
    let mut max_position = 0;

    for pass in 0..2 {
        let direction = if pass == 0 { ServoDirection::Clockwise } else { ServoDirection::Counterclockwise };
        println!(
            "Starting calibration pass {}, direction: {:?}",
            pass + 1,
            direction
        );

        servo.set_speed(servo_id, calibration_speed, direction)?;

        loop {
            if !running.load(Ordering::SeqCst) {
                println!("Calibration interrupted. Stopping servo...");
                servo.set_speed(servo_id, 0, ServoDirection::Clockwise)?;
                return Ok(());
            }

            let info = servo.read_info(servo_id)?;
            let position = info.current_location;
            let current = info.current_current as f32 * 6.5 / 100.0;

            if current > current_threshold {
                println!("Current: {:.2}, Threshold: {:.2}", current, current_threshold);
                println!("Current threshold reached at position {}", position);

                // Stop
                servo.set_speed(servo_id, 0, direction)?;
                sleep(Duration::from_millis(100));

                // Save the angle and position
                if pass == 0 {
                    min_angle = info.current_angle;
                    min_position = position;
                } else {
                    max_angle = info.current_angle;
                    max_position = position;
                }

                break;
            }

            sleep(Duration::from_millis(10));
        }

        if pass < 1 {
            println!("Changing direction for next calibration pass...");
            sleep(Duration::from_millis(500));
        }
    }

    // Record the calibration data
    let record = CalibrationRecord {
        servo_id,
        iteration: 1, // You might want to track this across multiple calibrations
        min_angle,
        max_angle,
        min_position,
        max_position,
    };
    record_calibration(&record)?;

    servo.set_speed(servo_id, calibration_speed, ServoDirection::Clockwise)?;
    sleep(Duration::from_millis(100));
    servo.set_speed(servo_id, 0, ServoDirection::Clockwise)?;

    // Ensure max_angle > min_angle
    let min_angle = max_backward;
    let mut max_angle = max_forward;
    if max_angle <= min_angle {
        max_angle += 4096;
    }

    let center_distance = (max_angle - min_angle) / 2;
    // Calculate offset
    let offset = min_angle + center_distance - 2048;

    // Convert offset to 12-bit signed value
    let offset_value = if offset < 0 {
        (offset & 0x7FF) as u16 | 0x800 // Set sign bit
    } else {
        (offset & 0x7FF) as u16
    };

    // unlock EEPROM
    servo.write(servo_id, ServoRegister::LockMark, &[0])?;
    sleep(Duration::from_millis(10));

    servo.write(servo_id, ServoRegister::OperationMode, &[0])?;
    sleep(Duration::from_millis(10));
    println!("Switched servo to mode 0.");

    servo.write_servo_memory(
        servo_id,
        ServoRegister::PositionCorrection,
        offset_value,
    )?;

    sleep(Duration::from_millis(10));
    // Write servo limits to memory
    servo.write_servo_memory(servo_id, ServoRegister::MinAngleLimit, min_angle as u16)?;
    sleep(Duration::from_millis(10));
    servo.write_servo_memory(servo_id, ServoRegister::MaxAngleLimit, max_angle as u16)?;
    sleep(Duration::from_millis(10));
    servo.write(servo_id, ServoRegister::LockMark, &[1])?; // lock EEPROM

    println!("Successfully wrote calibration data to EEPROM.");

    println!("Calibration complete.");
    println!("Offset: {}", offset);
    println!("Min Angle: {}", min_angle);
    println!("Max Angle: {}", max_angle);

    sleep(Duration::from_millis(100));

    // let position_data = [(2048 & 0xFF) as u8, ((2048 >> 8) & 0xFF) as u8];
    // servo.write(servo_id, ServoRegister::TargetLocation, &position_data)?;

    println!("Wrote servo limits to memory:");
    println!("Min Angle: {}", min_angle);
    println!("Max Angle: {}", max_angle);

    println!("Moving servo to middle 2048");

    sleep(Duration::from_secs(1));

    println!("Calibration and positioning complete.");

    servo.enable_readout()?;

    // Disable torque
    let torque_data = 0u8;
    match servo.write(servo_id, ServoRegister::TorqueSwitch, &[torque_data]) {
        Ok(_) => println!("Torque disabled successfully."),
        Err(e) => println!("Failed to disable torque. Error: {}", e),
    }
    Ok(())
}

pub fn calibrate_servo(servo: &Servo, servo_id: u8, running: &Arc<AtomicBool>, calibration_speed: u16, current_threshold: f32) -> Result<()> {
    println!("Starting servo calibration for ID: {}", servo_id);
    println!("Calibration speed: {}", calibration_speed);
    println!("Current threshold: {} mA", current_threshold);

    servo.disable_readout()?;
    servo.set_mode(servo_id, ServoMode::ConstantSpeed)?; // Set to continuous mode

    let mut max_forward = 0;
    let mut max_backward = 0;

    for pass in 0..2 {
        let direction = if pass == 0 { ServoDirection::Clockwise } else { ServoDirection::Counterclockwise };
        println!(
            "Starting calibration pass {}, direction: {:?}",
            pass + 1,
            direction
        );

        servo.set_speed(servo_id, calibration_speed, direction)?;

        loop {
            if !running.load(Ordering::SeqCst) {
                println!("Calibration interrupted. Stopping servo...");
                servo.set_speed(servo_id, 0, ServoDirection::Clockwise)?;
                return Ok(());
            }

            let info = servo.read_info(servo_id)?;
            let position = info.current_location;
            let mut current = info.current_current as f32 * 6.5 / 100.0;

            if current > current_threshold {
                println!("Current: {:.2}, Threshold: {:.2}", current, current_threshold);
                println!("Current threshold reached at position {}", position);

                // Stop
                servo.set_speed(servo_id, 0, direction)?;
                sleep(Duration::from_millis(100));

                println!("Backing off");
                // Back off
                servo.set_speed(servo_id, calibration_speed, opposite_direction(direction))?;
                sleep(Duration::from_millis(350));

                // Stop after backoff
                servo.set_speed(servo_id, 0, opposite_direction(direction))?;
                sleep(Duration::from_millis(100));
                println!("Backing off complete");
                let info = servo.read_info(servo_id)?;
                println!(
                    "Calibration complete for this direction. Final position: {}",
                    info.current_location
                );


                if direction == ServoDirection::Clockwise {
                    println!("direction = clockwise");
                    max_forward = info.current_location;
                    println!(
                        "Forward calibration complete. Max position: {}",
                        max_forward
                    );
                } else {
                    println!("direction = counterclockwise");
                    max_backward = info.current_location;
                    println!(
                        "Backward calibration complete. Max position: {}",
                        max_backward
                    );
                }

                println!("max_forward = {}", max_forward);
                println!("max_backward = {}", max_backward);
                break;
            }

            sleep(Duration::from_millis(10));
        }

        if pass < 1 {
            println!("Changing direction for next calibration pass...");
            sleep(Duration::from_millis(500));
        }
    }

    servo.set_speed(servo_id, calibration_speed, ServoDirection::Clockwise)?;
    sleep(Duration::from_millis(100));
    servo.set_speed(servo_id, 0, ServoDirection::Clockwise)?;

    // Ensure max_angle > min_angle
    let min_angle = max_backward;
    let mut max_angle = max_forward;
    if max_angle <= min_angle {
        max_angle += 4096;
    }

    let center_distance = (max_angle - min_angle) / 2;
    // Calculate offset
    let offset = min_angle + center_distance - 2048;

    // Convert offset to 12-bit signed value
    let offset_value = if offset < 0 {
        (offset & 0x7FF) as u16 | 0x800 // Set sign bit
    } else {
        (offset & 0x7FF) as u16
    };

    // unlock EEPROM
    servo.write(servo_id, ServoRegister::LockMark, &[0])?;
    sleep(Duration::from_millis(10));

    servo.write(servo_id, ServoRegister::OperationMode, &[0])?;
    sleep(Duration::from_millis(10));
    println!("Switched servo to mode 0.");

    servo.write_servo_memory(
        servo_id,
        ServoRegister::PositionCorrection,
        offset_value,
    )?;

    sleep(Duration::from_millis(10));
    // Write servo limits to memory
    servo.write_servo_memory(servo_id, ServoRegister::MinAngleLimit, min_angle as u16)?;
    sleep(Duration::from_millis(10));
    servo.write_servo_memory(servo_id, ServoRegister::MaxAngleLimit, max_angle as u16)?;
    sleep(Duration::from_millis(10));
    // lock EEPROM
    servo.write(servo_id, ServoRegister::LockMark, &[1])?;

    println!("Successfully wrote calibration data to EEPROM.");

    println!("Calibration complete.");
    println!("Offset: {}", offset);
    println!("Min Angle: {}", min_angle);
    println!("Max Angle: {}", max_angle);

    sleep(Duration::from_millis(100));

    // let position_data = [(2048 & 0xFF) as u8, ((2048 >> 8) & 0xFF) as u8];
    // servo.write(servo_id, ServoRegister::TargetLocation, &position_data)?;

    println!("Wrote servo limits to memory:");
    println!("Min Angle: {}", min_angle);
    println!("Max Angle: {}", max_angle);

    println!("Moving servo to middle 2048");

    sleep(Duration::from_secs(1));

    println!("Calibration and positioning complete.");

    servo.enable_readout()?;

    // Disable torque
    let torque_data = 0u8;
    match servo.write(servo_id, ServoRegister::TorqueSwitch, &[torque_data]) {
        Ok(_) => println!("Torque disabled successfully."),
        Err(e) => println!("Failed to disable torque. Error: {}", e),
    }
    Ok(())
}

fn opposite_direction(direction: ServoDirection) -> ServoDirection {
    match direction {
        ServoDirection::Clockwise => ServoDirection::Counterclockwise,
        ServoDirection::Counterclockwise => ServoDirection::Clockwise,
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!("Starting auto calibration for servo ID: {}", args.servo_id);
    println!("Calibration speed: {}", args.calibration_speed);
    println!("Current threshold: {} mA", args.current_threshold);

    let servo = Arc::new(Servo::new()?);
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        println!("\nInterrupt signal received. Stopping calibration...");
        r.store(false, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    let result = auto_calibrate_servo(&servo, args.servo_id, &running, args.calibration_speed, args.current_threshold);

    if !running.load(Ordering::SeqCst) {
        println!("Calibration was interrupted. Cleaning up...");
        servo.set_speed(args.servo_id, 0, ServoDirection::Clockwise)?; // Stop the servo
        servo.enable_readout()?;
    }

    result?; // This will propagate any error from auto_calibrate_servo, including recording errors

    Ok(())
}

fn end_calibration(s: &mut cursive::Cursive, servo_id: u8, servo: Arc<Servo>) {
    let min_pos = CALIBRATION_POSITION.load(Ordering::Relaxed);
    if min_pos < 0 {
        s.add_layer(Dialog::info("Please start calibration first by pressing '['"));
        return;
    }

    let mut max_pos = CURRENT_POSITION.load(Ordering::Relaxed);

    if max_pos < min_pos {
        max_pos += 4096;
    }

    let offset_value = min_pos + (max_pos - min_pos) / 2 - 2048;

    // Convert offset to 12-bit signed value
    let offset_value = if offset_value < 0 {
        offset_value.abs() as u16 | 0x800 // (set negative)
    } else {
        if offset_value > 2048 {
            (offset_value - 4096).abs() as u16 | 0x800
        } else {
            offset_value as u16
        }
    };

    // Calculate new limits
    let min_angle = 2048 - (max_pos - min_pos) / 2;
    let max_angle = 2048 + (max_pos - min_pos) / 2;

    // Write new values to EEPROM
    if let Err(e) = write_calibration_to_eeprom(servo_id, &servo, offset_value, min_angle, max_angle) {
        s.add_layer(Dialog::info(format!("Error writing calibration to EEPROM: {}", e)));
        return;
    }

    // Update the UI
    s.call_on_name("MinAngle", |view: &mut TextView| {
        view.set_content(format!("Min Angle: {}", min_angle));
    });
    s.call_on_name("MaxAngle", |view: &mut TextView| {
        view.set_content(format!("Max Angle: {}", max_angle));
    });
    s.call_on_name("Offset", |view: &mut TextView| {
        view.set_content(format!("Offset: {}", offset_value));
    });

    CALIBRATION_POSITION.store(-1, Ordering::Relaxed);
    s.add_layer(Dialog::info(format!("Calibration completed for servo {}. New offset: {}", servo_id, offset_value)));
}

fn record_calibration(record: &CalibrationRecord) -> Result<()> {
    let file_name = format!("servo_{}_calibration.txt", record.servo_id);
    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(file_name)?;

    writeln!(
        file,
        "{},{},{},{},{},{}",
        record.servo_id,
        record.iteration,
        record.min_angle,
        record.max_angle,
        record.min_position,
        record.max_position
    )?;

    Ok(())
}
