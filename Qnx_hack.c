//client
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/ioctl.h>

#include <hw/i2c.h>
#include "common.h"

sensor_data_t shared_data;
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

int coid;
FILE *serial_port;

#define I2C_DEV "/dev/i2c1"
#define GPIO_DEV "/dev/gpio"

// ---------------- I2C HELPERS ----------------

int i2c_read(int fd, uint8_t slave_addr, uint8_t reg, uint8_t *buf, int len)
{
    i2c_sendrecv_t msg;

    msg.slave.addr = slave_addr;
    msg.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.send_len   = 1;
    msg.send_buf   = &reg;
    msg.recv_len   = len;
    msg.recv_buf   = buf;
    msg.stop       = 1;

    return devctl(fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL);
}

// ---------------- MPU6050 ----------------

void read_mpu6050(float *ax, float *ay, float *az)
{
    int fd = open(I2C_DEV, O_RDWR);
    if(fd < 0) return;

    uint8_t data[6];

    if(i2c_read(fd, 0x68, 0x3B, data, 6) == EOK)
    {
        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];

        *ax = raw_x / 16384.0;
        *ay = raw_y / 16384.0;
        *az = raw_z / 16384.0;
    }

    close(fd);
}

// ---------------- BMP280 ----------------

float read_bmp280()
{
    int fd = open(I2C_DEV, O_RDWR);
    if(fd < 0) return 0;

    uint8_t data[3];
    float pressure = 0;

    if(i2c_read(fd, 0x76, 0xF7, data, 3) == EOK)
    {
        int raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        pressure = raw / 256.0;
    }

    close(fd);
    return pressure;
}

// ---------------- DS3231 ----------------

int bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

void read_ds3231(char *buffer)
{
    int fd = open(I2C_DEV, O_RDWR);
    if(fd < 0) return;

    uint8_t data[3];

    if(i2c_read(fd, 0x68, 0x00, data, 3) == EOK)
    {
        int sec  = bcd_to_dec(data[0]);
        int min  = bcd_to_dec(data[1]);
        int hour = bcd_to_dec(data[2]);

        sprintf(buffer, "%02d:%02d:%02d", hour, min, sec);
    }

    close(fd);
}

// ---------------- VIBRATION GPIO ----------------

int read_vibration_gpio()
{
    int fd = open(GPIO_DEV, O_RDONLY);
    if(fd < 0) return 0;

    int value = 0;

    devctl(fd, 17, &value, sizeof(value), NULL);

    close(fd);

    return value;
}

// ---------------- THREADS ----------------

void* mpu_thread(void *arg)
{
    while(1){
        pthread_mutex_lock(&data_mutex);

        read_mpu6050(&shared_data.acc_x,
                     &shared_data.acc_y,
                     &shared_data.acc_z);

        pthread_mutex_unlock(&data_mutex);

        sleep(1);
    }
    return NULL;
}

void* bmp_thread(void *arg)
{
    while(1){
        pthread_mutex_lock(&data_mutex);

        shared_data.pressure = read_bmp280();

        pthread_mutex_unlock(&data_mutex);

        sleep(1);
    }
    return NULL;
}

void* vibration_thread(void *arg)
{
    while(1){
        pthread_mutex_lock(&data_mutex);

        shared_data.vibration = read_vibration_gpio();

        pthread_mutex_unlock(&data_mutex);

        sleep(1);
    }
    return NULL;
}

// ---------------- MAIN ----------------

int main()
{
    pthread_t t1, t2, t3;
    reply_msg_t reply;

    serial_port = fopen("/dev/ser1", "w");
    if(!serial_port){
        perror("serial open");
        return -1;
    }

    coid = name_open(SERVER_NAME, 0);
    if(coid == -1){
        perror("name_open");
        return -1;
    }

    pthread_create(&t1, NULL, mpu_thread, NULL);
    pthread_create(&t2, NULL, bmp_thread, NULL);
    pthread_create(&t3, NULL, vibration_thread, NULL);

    while(1){
        pthread_mutex_lock(&data_mutex);

        read_ds3231(shared_data.timestamp);

        MsgSend(coid,
                &shared_data,
                sizeof(shared_data),
                &reply,
                sizeof(reply));

        fprintf(serial_port,
                "Time:%s X=%.2f Y=%.2f Z=%.2f Pressure=%.2f Vib=%d Reply=%s\n",
                shared_data.timestamp,
                shared_data.acc_x,
                shared_data.acc_y,
                shared_data.acc_z,
                shared_data.pressure,
                shared_data.vibration,
                reply.message);

        fflush(serial_port);

        pthread_mutex_unlock(&data_mutex);

        sleep(1);
    }

    fclose(serial_port);
    name_close(coid);

    return 0;
}



#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <stdint.h>
#include <hw/inout.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <hw/i2c.h>

#include "common.h"

#define I2C_DEV "/dev/i2c1"

#define MPU_ADDR  0x69
#define BMP_ADDR  0x76
#define RTC_ADDR  0x68

#define GPIO_BASE 0xFE200000
#define GPIO_LEN  0x1000

sensor_data_t shared_data;
pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

int coid;
FILE *serial_port;
int i2c_fd;

/* ---------------- I2C READ ---------------- */
int i2c_read(int fd, uint8_t slave_addr, uint8_t reg, uint8_t *data, int len)
{
    struct {
        i2c_sendrecv_t hdr;
        uint8_t buf[17];
    } msg;

    memset(&msg, 0, sizeof(msg));

    msg.hdr.slave.addr = slave_addr;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.send_len   = 1;
    msg.hdr.recv_len   = len;
    msg.hdr.stop       = 1;

    msg.buf[0] = reg;

    if(devctl(fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL) != EOK)
    {
        perror("i2c_read failed");
        return -1;
    }

    memcpy(data, &msg.buf[1], len);
    return 0;
}

/* ---------------- I2C WRITE ---------------- */
int i2c_write(int fd, uint8_t slave_addr, uint8_t reg, uint8_t value)
{
    struct {
        i2c_send_t hdr;
        uint8_t buf[2];
    } msg;

    memset(&msg, 0, sizeof(msg));

    msg.hdr.slave.addr = slave_addr;
    msg.hdr.slave.fmt  = I2C_ADDRFMT_7BIT;
    msg.hdr.len        = 2;
    msg.hdr.stop       = 1;

    msg.buf[0] = reg;
    msg.buf[1] = value;

    if(devctl(fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL) != EOK)
    {
        perror("i2c_write failed");
        return -1;
    }

    return 0;
}

/* ---------------- SENSOR INIT ---------------- */
int init_mpu6050()
{
    return i2c_write(i2c_fd, MPU_ADDR, 0x6B, 0x00);
}

int init_bmp280()
{
    return i2c_write(i2c_fd, BMP_ADDR, 0xF4, 0x27);
}

/* ---------------- MPU6050 READ ---------------- */
void read_mpu6050(float *ax, float *ay, float *az)
{
    uint8_t data[6];

    if(i2c_read(i2c_fd, MPU_ADDR, 0x3B, data, 6) == 0)
    {
        int16_t raw_x = (data[0] << 8) | data[1];
        int16_t raw_y = (data[2] << 8) | data[3];
        int16_t raw_z = (data[4] << 8) | data[5];

        *ax = raw_x / 16384.0f;
        *ay = raw_y / 16384.0f;
        *az = raw_z / 16384.0f;
    }
}

/* ---------------- BMP280 READ ---------------- */
float read_bmp280()
{
    uint8_t data[3];

    if(i2c_read(i2c_fd, BMP_ADDR, 0xF7, data, 3) == -1)
        return -1;

    int32_t raw_press = ((int32_t)data[0] << 12) |
                        ((int32_t)data[1] << 4)  |
                        ((int32_t)data[2] >> 4);

    return raw_press / 256.0f;
}

/* ---------------- DS3231 READ ---------------- */
int bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

void read_ds3231(char *buffer)
{
    uint8_t data[3];

    if(i2c_read(i2c_fd, RTC_ADDR, 0x00, data, 3) == 0)
    {
        int sec  = bcd_to_dec(data[0] & 0x7F);
        int min  = bcd_to_dec(data[1] & 0x7F);
        int hour = bcd_to_dec(data[2] & 0x3F);

        sprintf(buffer, "%02d:%02d:%02d", hour, min, sec);
    }
    else
    {
        sprintf(buffer, "ERR");
    }
}

/* ---------------- VIBRATION GPIO ---------------- */
int read_vibration_gpio()
{
    uintptr_t gpio_base = mmap_device_io(GPIO_LEN, GPIO_BASE);
    if(gpio_base == MAP_DEVICE_FAILED)
    {
        perror("GPIO map failed");
        return -1;
    }

    uint32_t level = in32(gpio_base + 0x34);
    int value = (level & (1 << 17)) ? 1 : 0;

    munmap_device_io(gpio_base, GPIO_LEN);

    return value;
}

/* ---------------- THREADS ---------------- */
void* mpu_thread(void *arg)
{
    while(1)
    {
        pthread_mutex_lock(&data_mutex);
        read_mpu6050(&shared_data.acc_x, &shared_data.acc_y, &shared_data.acc_z);
        pthread_mutex_unlock(&data_mutex);
        sleep(1);
    }
    return NULL;
}

void* bmp_thread(void *arg)
{
    while(1)
    {
        pthread_mutex_lock(&data_mutex);
        shared_data.pressure = read_bmp280();
        pthread_mutex_unlock(&data_mutex);
        sleep(1);
    }
    return NULL;
}

void* vibration_thread(void *arg)
{
    while(1)
    {
        pthread_mutex_lock(&data_mutex);
        shared_data.vibration = read_vibration_gpio();
        pthread_mutex_unlock(&data_mutex);
        sleep(1);
    }
    return NULL;
}

/* ---------------- MAIN ---------------- */
int main()
{
    pthread_t t1, t2, t3;
    pthread_attr_t attr1, attr2, attr3;
    struct sched_param param;
    reply_msg_t reply;

    i2c_fd = open(I2C_DEV, O_RDWR);
    if(i2c_fd < 0)
    {
        perror("Failed to open I2C");
        return -1;
    }

    if(init_mpu6050() == -1)
    {
        printf("MPU6050 init failed\n");
        return -1;
    }

    if(init_bmp280() == -1)
    {
        printf("BMP280 init failed\n");
        return -1;
    }

    serial_port = fopen("/dev/ser1", "w");
    if(!serial_port)
    {
        perror("serial open");
        return -1;
    }

    coid = name_open(SERVER_NAME, 0);
    if(coid == -1)
    {
        perror("name_open");
        return -1;
    }

    pthread_attr_init(&attr1);
    pthread_attr_init(&attr2);
    pthread_attr_init(&attr3);

    pthread_attr_setinheritsched(&attr1, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setinheritsched(&attr2, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setinheritsched(&attr3, PTHREAD_EXPLICIT_SCHED);

    pthread_attr_setschedpolicy(&attr1, SCHED_RR);
    pthread_attr_setschedpolicy(&attr2, SCHED_RR);
    pthread_attr_setschedpolicy(&attr3, SCHED_RR);

    param.sched_priority = 10;
    pthread_attr_setschedparam(&attr1, &param);

    param.sched_priority = 20;
    pthread_attr_setschedparam(&attr2, &param);

    param.sched_priority = 30;
    pthread_attr_setschedparam(&attr3, &param);

    pthread_create(&t1, &attr1, mpu_thread, NULL);
    pthread_create(&t2, &attr2, bmp_thread, NULL);
    pthread_create(&t3, &attr3, vibration_thread, NULL);

    while(1)
    {
        pthread_mutex_lock(&data_mutex);

        read_ds3231(shared_data.timestamp);

        if(MsgSend(coid, &shared_data, sizeof(shared_data),
                   &reply, sizeof(reply)) != -1)
        {
        	printf("Server Reply: %s\n", reply.message);
            fprintf(serial_port,
                "Time:%s X=%.2f Y=%.2f Z=%.2f Pressure=%.2f Vib=%d Reply=%s\n",
                shared_data.timestamp,
                shared_data.acc_x,
                shared_data.acc_y,
                shared_data.acc_z,
                shared_data.pressure,
                shared_data.vibration,
                reply.message);

            fflush(serial_port);
        }

        pthread_mutex_unlock(&data_mutex);
        sleep(1);
    }

    close(i2c_fd);
    fclose(serial_port);
    name_close(coid);

    return 0;
}









//server
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "common.h"

/* -------- THRESHOLDS -------- */
#define PRESSURE_NORMAL_MAX   1050.0
#define PRESSURE_WARNING_MAX  1100.0

#define MPU_ACCEL_NORMAL      2.0
#define MPU_ACCEL_WARNING     5.0

#define VIB_ALERT             1

int main(void)
{
    name_attach_t *attach;
    int rcvid;
    sensor_data_t msg;
    reply_msg_t reply;

    attach = name_attach(NULL, SERVER_NAME, 0);
    if (attach == NULL)
    {
        perror("name_attach failed");
        return EXIT_FAILURE;
    }

    printf("SERVER STARTED...\n");

    while (1)
    {
        rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1)
            continue;

        printf("TIME:%s Pressure=%.2f MPU=(%.2f, %.2f, %.2f) Vib=%d\n",
               msg.timestamp,
               msg.pressure,
               msg.acc_x,
               msg.acc_y,
               msg.acc_z,
               msg.vibration);

        memset(&reply, 0, sizeof(reply));
        int flag = 0;   // 0=Healthy, 1=Warning, 2=Critical

        /* -------- PRESSURE CHECK -------- */
        if (msg.pressure < 0)
        {
            strcat(reply.message, "PRESSURE ERROR ");
            flag = 2;
        }
        else if (msg.pressure < PRESSURE_NORMAL_MAX)
        {
            strcat(reply.message, "PRESSURE NORMAL ");
        }
        else if (msg.pressure <= PRESSURE_WARNING_MAX)
        {
            strcat(reply.message, "PRESSURE WARNING ");
            flag = 1;
        }
        else
        {
            strcat(reply.message, "PRESSURE CRITICAL ");
            flag = 2;
        }

        /* -------- MPU CHECK -------- */
        float accel_mag = sqrt(
            (msg.acc_x * msg.acc_x) +
            (msg.acc_y * msg.acc_y) +
            (msg.acc_z * msg.acc_z)
        );

        if (accel_mag < MPU_ACCEL_NORMAL)
        {
            strcat(reply.message, "MPU NORMAL ");
        }
        else if (accel_mag < MPU_ACCEL_WARNING)
        {
            strcat(reply.message, "MPU WARNING ");
            if (flag < 1) flag = 1;
        }
        else
        {
            strcat(reply.message, "MPU CRITICAL ");
            flag = 2;
        }

        /* -------- VIBRATION CHECK -------- */
        if (msg.vibration < 0)
        {
            strcat(reply.message, "VIB SENSOR ERROR ");
            flag = 2;
        }
        else if (msg.vibration == VIB_ALERT)
        {
            strcat(reply.message, "VIBRATION DETECTED ");
            if (flag < 1) flag = 1;
        }
        else
        {
            strcat(reply.message, "NO VIBRATION ");
        }

        /* -------- FINAL STATUS -------- */
        reply.status = flag;

        if (flag == 0)
            strcat(reply.message, "SYSTEM HEALTHY");
        else if (flag == 1)
            strcat(reply.message, "SYSTEM WARNING");
        else
            strcat(reply.message, "SYSTEM CRITICAL");

        MsgReply(rcvid, EOK, &reply, sizeof(reply));
    }

    name_detach(attach, 0);
    return EXIT_SUCCESS;
}
