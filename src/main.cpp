#include "mbed.h"
#include "arm_math.h"

// ========= SERIAL ==========
UnbufferedSerial pc(USBTX, USBRX, 115200);
FileHandle* mbed::mbed_override_console(int) { return &pc; }

// ========= LEDS ============
DigitalOut led_tremor(LED1);
DigitalOut led_dysk(LED2);
DigitalOut led_freeze(LED3);

// ========= I2C + IMU ========
I2C i2c(PB_11, PB_10);

#define LSM6DSL_ADDR (0x6A << 1)

#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12

#define OUTX_L_XL   0x28
#define OUTY_L_XL   0x2A
#define OUTZ_L_XL   0x2C

#define OUTX_L_G    0x22
#define OUTY_L_G    0x24
#define OUTZ_L_G    0x26

void write_reg(uint8_t reg, uint8_t val) {
    char data[2] = { (char)reg, (char)val };
    i2c.write(LSM6DSL_ADDR, data, 2);
}

bool read_reg(uint8_t reg, uint8_t &val) {
    char r = reg;
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    val = r;
    return true;
}

int16_t read_axis(uint8_t low_addr) {
    uint8_t lo, hi;
    read_reg(low_addr, lo);
    read_reg(low_addr + 1, hi);
    return (int16_t)((hi << 8) | lo);
}

bool init_sensor() {
    uint8_t who;
    read_reg(WHO_AM_I, who);
    if (who != 0x6A) return false;

    write_reg(CTRL3_C, 0x44);  // BDU + auto-increment
    write_reg(CTRL1_XL, 0x40); // ACCEL: 52 Hz, ±2g
    write_reg(CTRL2_G,  0x40); // *** GYRO ON: 52 Hz, ±250 dps ***
    return true;
}

// ========= FFT PARAMETERS =========
#define SAMPLE_RATE 52
#define WINDOW_SEC  3
#define RAW_SAMPLES (SAMPLE_RATE * WINDOW_SEC)  // 156
#define FFT_SIZE    256

float accel_buf[RAW_SAMPLES];
float gyro_buf[RAW_SAMPLES];

int buf_idx = 0;

float fft_in[FFT_SIZE];
float fft_out[FFT_SIZE];
float fft_mag[FFT_SIZE/2];

arm_rfft_fast_instance_f32 rfft;

volatile bool sample_flag = false;

void tick_isr() { sample_flag = true; }

// ========= SAFE FLOAT PRINT =========
void print_float(const char *label, float v) {
    int ip = (int)v;
    int fp = (int)((v - ip) * 1000);
    if (fp < 0) fp = -fp;
    printf("%s%d.%03d", label, ip, fp);
}

// ========= MAIN =========
int main() {

    printf("Parkinson Real FFT Detector (Option C)\r\n");

    if (!init_sensor()) {
        printf("IMU init failed!\r\n");
        while (1);
    }

    arm_rfft_fast_init_f32(&rfft, FFT_SIZE);

    Ticker tick;
    tick.attach(&tick_isr, 1.0f / SAMPLE_RATE);

    Timer timer;
    timer.start();

    while (true) {

        // ======= SAMPLE DATA ========
        if (sample_flag) {
            sample_flag = false;

            // --- ACCEL ---
            int16_t x = read_axis(OUTX_L_XL);
            int16_t y = read_axis(OUTY_L_XL);
            int16_t z = read_axis(OUTZ_L_XL);
            float ax = x * 0.000061f;
            float ay = y * 0.000061f;
            float az = z * 0.000061f;
            float amag = sqrtf(ax*ax + ay*ay + az*az);

            // --- GYRO (OPTION C — MAIN FOR TREMOR/DYSK) ---
            int16_t gx = read_axis(OUTX_L_G);
            int16_t gy = read_axis(OUTY_L_G);
            int16_t gz = read_axis(OUTZ_L_G);
            float fgx = gx * 0.00875f;  // ±250 dps scale
            float fgy = gy * 0.00875f;
            float fgz = gz * 0.00875f;
            float gmag = sqrtf(fgx*fgx + fgy*fgy + fgz*fgz);

            accel_buf[buf_idx] = amag;
            gyro_buf[buf_idx]  = gmag;
            buf_idx++;
            if (buf_idx >= RAW_SAMPLES) buf_idx = 0;
        }

        // ======= PROCESS EVERY 3 SECONDS ========
        if (timer.elapsed_time().count() >= WINDOW_SEC * 1000000) {
            timer.reset();

            // ======= ACCEL FFT FOR WALK + FREEZE =======
            float mean = 0;
            for (int i=0; i < RAW_SAMPLES; i++) mean += accel_buf[i];
            mean /= RAW_SAMPLES;

            for (int i=0; i < RAW_SAMPLES; i++)
                fft_in[i] = accel_buf[i] - mean;
            for (int i=RAW_SAMPLES; i < FFT_SIZE; i++)
                fft_in[i] = 0.0f;

            arm_rfft_fast_f32(&rfft, fft_in, fft_out, 0);
            arm_cmplx_mag_f32(fft_out, fft_mag, FFT_SIZE/2);

            float hz_per_bin = (float)SAMPLE_RATE / FFT_SIZE;

            float walk = 0, fog = 0;
            for (int k=1; k < FFT_SIZE/2; k++) {
                float f = k * hz_per_bin;
                if (f >= 0.5f && f <= 3.0f) walk += fft_mag[k];
                if (f > 3.0f && f <= 8.0f)  fog  += fft_mag[k];
            }

            // ======= GYRO FFT FOR TREMOR + DYSK =======
            mean = 0;
            for (int i=0; i < RAW_SAMPLES; i++) mean += gyro_buf[i];
            mean /= RAW_SAMPLES;

            for (int i=0; i < RAW_SAMPLES; i++)
                fft_in[i] = gyro_buf[i] - mean;
            for (int i=RAW_SAMPLES; i < FFT_SIZE; i++)
                fft_in[i] = 0.0f;

            arm_rfft_fast_f32(&rfft, fft_in, fft_out, 0);
            arm_cmplx_mag_f32(fft_out, fft_mag, FFT_SIZE/2);

            float tremor = 0, dysk = 0;
            for (int k=1; k < FFT_SIZE/2; k++) {
                float f = k * hz_per_bin;
                if (f >= 3.0f && f <= 5.0f) tremor += fft_mag[k];
                if (f > 5.0f && f <= 7.0f)  dysk   += fft_mag[k];
            }

            float fog_ratio = fog / (walk + 0.0001f);

            // ======= LOGIC =======
            bool tremor_present = tremor > 5.0f;
            bool dysk_present   = dysk   > 5.0f;
            bool low_walk       = walk < 5.0f;

            bool freezing = false;
            if (fog_ratio > 3.0f && low_walk && !dysk_present)
                freezing = true;

            led_tremor = 0;
            led_dysk = 0;
            led_freeze = 0;

            if (freezing) {
                led_freeze = 1;
                if (tremor_present) led_tremor = 1;
            }
            else {
                if (low_walk && tremor_present && tremor > dysk * 1.2f)
                    led_tremor = 1;

                if (low_walk && dysk_present && dysk > tremor * 1.2f)
                    led_dysk = 1;
            }

            // ======= PRINT OUTPUT =======
            print_float("Tremor=", tremor); printf("  ");
            print_float("Dysk=", dysk);     printf("  ");
            print_float("FogRatio=", fog_ratio); printf("  ");
            print_float("Walk=", walk);     printf("  \r\n");

            printf("Freeze=%d  ", freezing);
            printf("Is tremor?=%d  ", (low_walk && tremor_present && tremor > dysk * 1.2f));
            printf("Is dysk?=%d\r\n", (low_walk && dysk_present && dysk > tremor * 1.2f));
        }
    }
}
