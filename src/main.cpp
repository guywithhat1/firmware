#include <Arduino.h>
#include <SPI.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "sensors/RefSystem.hpp"
#include "filters/pid_filter.hpp"


// ENCODER
#define MT6835_OP_READ  0b0011
#define MT6835_OP_WRITE 0b0110
#define MT6835_OP_PROG  0b1100
#define MT6835_OP_ZERO  0b0101
#define MT6835_OP_ANGLE 0b1010
#define MT6835_CMD_MASK  0b111100000000000000000000
#define MT6835_ADDR_MASK 0b000011111111111100000000
#define MT6835_DATA_MASK 0b000000000000000011111111
#define MT6835_CPR 2097152
#define MT6835_STATUS_OVERSPEED 0x01
#define MT6835_STATUS_WEAKFIELD 0x02
#define MT6835_STATUS_UNDERVOLT 0x04
#define MT6835_CRC_ERROR 0x08
#define MT6835_WRITE_ACK 0x55
#define MT6835_REG_USERID 0x001
#define MT6835_REG_ANGLE1 0x003
#define MT6835_REG_ANGLE2 0x004
#define MT6835_REG_ANGLE3 0x005
#define MT6835_REG_ANGLE4 0x006
#define MT6835_REG_ABZ_RES1 0x007
#define MT6835_REG_ABZ_RES2 0x008
#define MT6835_REG_ZERO1 0x009
#define MT6835_REG_ZERO2 0x00A
#define MT6835_REG_OPTS0 0x00A
#define MT6835_REG_OPTS1 0x00B
#define MT6835_REG_OPTS2 0x00C
#define MT6835_REG_OPTS3 0x00D
#define MT6835_REG_OPTS4 0x00E
#define MT6835_REG_OPTS5 0x011
#define MT6835_REG_NLC_BASE 0x013
#define MT6835_REG_CAL_STATUS 0x113
#define MT6835_BITORDER MSBFIRST

#define MOUSE_SENSITIVITY 0.001


// pitch zero 0.6195919 rad
// yaw zero   4.99714899 rad
#define PITCH_ZERO_ANGLE 0.67928569282
#define YAW_ZERO_ANGLE   4.99714899

// declare any 'global' variables here
DR16 dr16;
rm_CAN can;
RefSystem ref;

static SPISettings settings(1000000, MT6835_BITORDER, SPI_MODE3);

Timer loop_timer;

// PID CONTROLLERS - THIS IS GROSS
PIDFilter drive;
PIDFilter feeder;
PIDFilter flywheel;
PIDFilter pitch;
PIDFilter yaw;

// DONT put anything else in this function. It is not a setup function
void print_logo() {
    if (Serial) {
        Serial.println("TEENSY SERIAL START\n\n");
        Serial.print("\033[1;33m");
        Serial.println("                  .:^!?!^.                        ");
        Serial.println("           .:~!?JYYYJ?7?Y5Y7!!.                   ");
        Serial.println("         :?5YJ?!~:.      ^777YP?.                 ");
        Serial.println("         5G~                  ~YP?:               ");
        Serial.println("         7P5555Y:               ^YP?:....         ");
        Serial.println("        ~55J7~^.   ..    .        ^JYYYYYYYYYJJ!. ");
        Serial.println("        YG^     !Y5555J:^PJ    Y5:      ...::^5G^ ");
        Serial.println("       :GY    .YG?^..^~ ~GY    5G^ ^!~~^^^!!~7G?  ");
        Serial.println(" .!JYYY5G!    7BJ       ~GY    5G^ ~??JJJY555GP!  ");
        Serial.println("^55!^:.^~.    ^PP~   .: ^GP:  ^PP:           :7PY.");
        Serial.println("YG^            :JP5YY55: ~YP55PY^              ~GJ");
        Serial.println("?G~      .?7~:   .^~~^.    .^:.                :G5");
        Serial.println(".5P^     7BYJ5YJ7^.                          .~5P^");
        Serial.println(" .JPJ!~!JP?  .:~?PP^            .:.    .^!JYY5Y!. ");
        Serial.println("   :!???!:       5P.         .!Y5YYYJ?Y5Y?!^:.    ");
        Serial.println("                 7G7        7GY!. .:~!^.          ");
        Serial.println("                  JG!      :G5                    ");
        Serial.println("                   7PY!^^~?PY:                    ");
        Serial.println("                    .!JJJJ?^                      ");
        Serial.print("\033[0m");
        Serial.println("\n\033[1;92mFW Ver. 2.1.0");
        Serial.printf("\nLast Built: %s at %s", __DATE__, __TIME__);
        Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}

float read_enc(int nCS) {
    uint8_t data[6] = {0}; // transact 48 bits
    data[0] = (MT6835_OP_ANGLE<<4);
    data[1] = MT6835_REG_ANGLE1;
    SPI.beginTransaction(settings);
    digitalWrite(nCS, LOW);
    SPI.transfer(data, 6);
    digitalWrite(nCS, HIGH);
    SPI.endTransaction();
    int raw_angle = (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
    float radians = raw_angle / (float)MT6835_CPR * (3.14159265*2.0);
    float degrees = radians * (180/3.14159265);
    // Serial.printf("nCS: %d      %d raw      %0.8f degrees      %0.8f radians\n", nCS, raw_angle, degrees, radians);
    return radians;
}

float wrap_angle(float angle) {
	while (angle >= 3.14159) angle -= 2 * PI;
	while (angle < -3.14159) angle += 2 * PI;
	return angle;
}

void rotate_2D(float* v, float* v_tf, float angle) {
	v_tf[0] = (v[0] * cos(angle)) - (v[1] * sin(angle));
	v_tf[1] = (v[0] * sin(angle)) + (v[1] * cos(angle));
}

// Master loop
int main() {
    Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    print_logo();

    // initialize any 'setup' functions here
    pinMode(13, OUTPUT);
    dr16.init();
    can.init();
    ref.init();

    // Encoder setup
    int nCS_yaw = 37; // 37 or 36 (enc 1, enc 2)
    int nCS_pitch = 36; // 37 or 36 (enc 1, enc 2)
    pinMode(nCS_yaw, OUTPUT);
    pinMode(nCS_pitch, OUTPUT);
    digitalWrite(nCS_yaw, HIGH);
    digitalWrite(nCS_pitch, HIGH);
    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");

    // PID Tuning
    drive.K[0] = 0.0005;

    feeder.K[0] = 0.02;
    feeder.K[2] = 0.005;

    flywheel.K[0] = 0.0008;

    yaw.K[0] = 0.005;

    pitch.K[0] = 0.0025;

    // PID Util Values
    float motor_speed;
    float output;
    float m_id;

    long long loopc;

    // main loop
    while (true) {
        loopc++;

        dr16.read();
        can.read();
        if (!(loopc % 1)) ref.read();

        float yaw_raw = read_enc(nCS_yaw);
        float yaw_ref = wrap_angle(yaw_raw - YAW_ZERO_ANGLE);
        float pitch_raw = read_enc(nCS_pitch);
        float pitch_ref = wrap_angle(pitch_raw - PITCH_ZERO_ANGLE);
        // Serial.printf("yaw enc: %f     pitch enc: %f\n", yaw_raw, pitch_raw);
        
        // Read DR16
        bool w_key = dr16.keys.w;
        bool a_key = dr16.keys.a;
        bool s_key = dr16.keys.s
        bool d_key = dr16.keys.d

        int mouse_x = dr16.get_mouse_x();
        int mouse_y = dr16.get_mouse_y();

        float drive_raw[2] = {0};
        drive_raw[0] = -dr16.get_l_stick_x();
        drive_raw[1] = -dr16.get_l_stick_y();
        float s = dr16.get_wheel();
        float drive_rot[2] = {0};
        rotate_2D(drive_raw, drive_rot, yaw_ref+(3.14159/4.0));
        float x = drive_rot[0] + w - s
        float y = drive_rot[1] + d - a;
        float pitch_js = dr16.get_r_stick_y() + (mouse_x * MOUSE_SENSITIVITY);
        float yaw_js = dr16.get_r_stick_x() + (mouse_y * MOUSE_SENSITIVITY);
       

        // Power limiting
        float power_buffer = ref.data.power_heat.buffer_energy;
        float power_limit_ratio = 1.0;
        float power_buffer_limit_thresh = 60.0;
	    float power_buffer_critical_thresh = 30.0;
        if (power_buffer < power_buffer_limit_thresh) {
            power_limit_ratio = constrain((power_buffer - power_buffer_critical_thresh) / power_buffer_limit_thresh, 0.0, 1.0);
        }
        Serial.println(power_buffer);

        // Drive 1
        m_id = 0;
        motor_speed = can.get_motor_attribute(CAN_1, m_id, MotorAttribute::SPEED);
        drive.setpoint = (-y+x+s) * 9000;
        drive.measurement = motor_speed;
        output = drive.filter(0.001) * power_limit_ratio;
        can.write_motor_norm(CAN_1, m_id, C620, output);

        // Drive 2
        m_id = 1;
        motor_speed = can.get_motor_attribute(CAN_1, m_id, MotorAttribute::SPEED);
        drive.setpoint = (-y-x+s) * 9000;
        drive.measurement = motor_speed;
        output = drive.filter(0.001) * power_limit_ratio;
        can.write_motor_norm(CAN_1, m_id, C620, output);

        // Drive 3
        m_id = 2;
        motor_speed = can.get_motor_attribute(CAN_1, m_id, MotorAttribute::SPEED);
        drive.setpoint = (y-x+s) * 9000;
        drive.measurement = motor_speed;
        output = drive.filter(0.001) * power_limit_ratio;
        can.write_motor_norm(CAN_1, m_id, C620, output);

        // Drive 4
        m_id = 3;
        motor_speed = can.get_motor_attribute(CAN_1, m_id, MotorAttribute::SPEED);
        drive.setpoint = (y+x+s) * 9000;
        drive.measurement = motor_speed;
        output = drive.filter(0.001) * power_limit_ratio;
        can.write_motor_norm(CAN_1, m_id, C620, output);

        // Yaw
        m_id = 4;
        motor_speed = can.get_motor_attribute(CAN_1, m_id, MotorAttribute::SPEED) * 0.05105105105;
        yaw.setpoint = yaw_js*300;
        yaw.measurement = motor_speed;
        output = yaw.filter(0.001, false);
        can.write_motor_norm(CAN_1, m_id, C620, output);
        m_id = 5;
        can.write_motor_norm(CAN_1, m_id, C620, output);

        // Pitch
        m_id = 0;
        float clampLow = 1.309; // 1.4
        float clampHigh = 1.88496; //1.7
        float pitchConstant = 0.17*sin(pitch_ref);
        motor_speed = can.get_motor_attribute(CAN_2, m_id, MotorAttribute::SPEED);
        pitch.setpoint = -pitch_js * 1000;
        pitch.measurement = motor_speed;
        output = pitch.filter(0.001, false) + pitchConstant;
        
        // if((pitch_ref > clampHigh && output>0) || (pitch_ref < clampLow && output < 0)){
        //     continue;
        // } else{
        can.write_motor_norm(CAN_2, m_id, C620, output);
         m_id = 1;
        can.write_motor_norm(CAN_2, m_id, C620, -output);
        // }

        // Feeder
        m_id = 4;
        motor_speed = can.get_motor_attribute(CAN_2, m_id, MotorAttribute::SPEED) / 36.0;
        feeder.setpoint = (dr16.get_r_switch() == 1 ? 100 : 0);
        feeder.measurement = motor_speed;
        float output = feeder.filter(0.001);
        can.write_motor_norm(CAN_2, m_id, C610, output);


        // Flywheel 1
        m_id = 2;
        motor_speed = can.get_motor_attribute(CAN_2, m_id, MotorAttribute::SPEED);
        flywheel.setpoint = (dr16.get_r_switch() != 2 ? -9000 : 0);
        flywheel.measurement = motor_speed;
        output = flywheel.filter(0.001);
        can.write_motor_norm(CAN_2, m_id, C620, output);

        // Flywheel 1
        m_id = 3;
        motor_speed = can.get_motor_attribute(CAN_2, m_id, MotorAttribute::SPEED);
        flywheel.setpoint = (dr16.get_r_switch() != 2 ? 9000 : 0);
        flywheel.measurement = motor_speed;
        output = flywheel.filter(0.001);
        can.write_motor_norm(CAN_2, m_id, C620, output);

        // Write to actuators
        if (!dr16.is_connected() || dr16.get_l_switch() == 1) {
            // SAFETY ON
            can.zero();
            can.zero_motors();
        } else if (dr16.is_connected() && dr16.get_l_switch() != 1) {
            // SAFETY OFF
            can.write();
        }

        // LED heartbeat
        millis() % 500 < 100 ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);

        // Keep the loop running at 1kHz
        loop_timer.delay_millis(1);
    }

    return 0;
}
