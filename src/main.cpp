/**
 * ESP32 双向无刷电调核心驱动代码（带航向角保持和深度控制）
 * PWM频率: 50Hz (周期20ms)
 * 脉宽范围: 1000us (全速正转) ~ 2000us (全速反转)
 * 中位停止: 1500us
 */

#include <Arduino.h>

// ======================================== 深度传感器（Serial1） ======================================== //
#define RX_PIN 40
#define TX_PIN 39
#define BAUD_RATE 115200

float zeroOffset = 0.24;         //默认偏移
float currentDepth = 0.0;        // 当前深度（米）
float targetDepth  = 0.30;       // 目标深度（米）

// 从一行字符串中提取深度值（米）
float parseDepth(const String& line) {
    int depthPos = line.indexOf("Depth:");
    if (depthPos == -1) return -1.0;
    int depthStart = depthPos + 6;
    int depthEnd = line.indexOf('m', depthStart);
    if (depthEnd == -1) return -1.0;
    String depthStr = line.substring(depthStart, depthEnd);
    return depthStr.toFloat();
}

// 非延时读取相对深度
void updateDepthReading() {
    while (Serial1.available()) {   
        String line = Serial1.readStringUntil('\n');
        line.trim();
        float rawDepth = parseDepth(line);
        if (rawDepth != -1.0) {currentDepth = rawDepth - zeroOffset;}
    }
}

// 读取带延时读取传感器深度（用于校准）
float readonereal(unsigned long timeoutMs = 0) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (Serial1.available()) {
            String line = Serial1.readStringUntil('\n');
            line.trim();
            float rawDepth = parseDepth(line);
            if (rawDepth != -1.0) {return rawDepth;}
        }
        delay(1);   
    }
    return -1.0;
}

// 校准零点偏移：采集 samples 个有效深度值，去掉最大最小值后取平均，要求 samples >= 3
// 返回 true 表示校准成功，false 表示失败
bool calibrateZeroOffset(int samples, float& offset) {
    int collected = 0;  
    // 动态数组存储所有有效值
    float* values = new float[samples];
    while (collected < samples) {
        float d = readonereal(500);       // 假设超时返回 -1
        if (d != -1.0f) {
            values[collected++] = d;
        }
    }
    
    // 找出最大值和最小值的索引
    int minIdx = 0, maxIdx = 0;
    for (int i = 1; i < samples; i++) {
        if (values[i] < values[minIdx]) minIdx = i;
        if (values[i] > values[maxIdx]) maxIdx = i;
    }
    
    // 求和时跳过最大值和最小值
    float sum = 0;
    int count = 0;
    for (int i = 0; i < samples; i++) {
        if (i == minIdx || i == maxIdx) continue;
        sum += values[i];
        count++;
    }
    offset = sum / count;
    
    delete[] values;
    return true;
}

// ========================================== IMU（Serial2） ========================================== //
#define CH0X0_RX  2
#define CH0X0_TX  1

float currentYaw = 0.0;  //当前航向
float targetYaw = 0.0;   //目标航向

void initCH0X0() {
    Serial2.begin(115200, SERIAL_8N1, CH0X0_RX, CH0X0_TX);
    delay(100);
    Serial2.print("LOG H191 ONTIME 0.02\r\n");
    Serial2.print("LOG ENABLE\r\n");
}

bool parseYawFromPacket(uint8_t *packet, float *yaw) {
    if (packet[6] != 0x91) return false;
    memcpy(yaw, &packet[62], 4);
    return true;
}

void updateYawReading() {
    static uint8_t buffer[82];
    static int bufIndex = 0;

    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        if (bufIndex == 0 && b == 0x5A) {
            buffer[bufIndex++] = b;
        } else if (bufIndex == 1 && b == 0xA5) {
            buffer[bufIndex++] = b;
        } else if (bufIndex > 1) {
            buffer[bufIndex++] = b;
            if (bufIndex >= 82) {
                if (buffer[2] == 0x4C && buffer[3] == 0x00) {
                    parseYawFromPacket(buffer, &currentYaw);
                }
                bufIndex = 0;
            }
        } else {
            bufIndex = 0;
        }
    }
}

// ========================================== GPS（UART0） ========================================== //
#define GPS_RX_PIN 13   // ESP32 接 GPS_TX
#define GPS_TX_PIN 14   // ESP32 接 GPS_RX
#define GPS_BAUD   9600

HardwareSerial GPS_Serial(0);

// 目标点：浙大城市学院北校区南门
const double TARGET_LAT = 30.3286877;
const double TARGET_LON = 120.1494317;

// 最新GPS定位结果
double gpsLat = 0.0;
double gpsLon = 0.0;
bool gpsHasFix = false;
unsigned long lastGpsFixTime = 0;

// GPS行缓存
char gpsLineBuffer[128] = {0};
size_t gpsBufferIdx = 0;

// 经纬度字符串转十进制
double nmeaToDecimal(const char* raw, char dir) {
    double val = atof(raw);
    int degrees = (int)(val / 100);
    double minutes = val - degrees * 100;
    double decimal = degrees + minutes / 60.0;

    if (dir == 'S' || dir == 'W') decimal = -decimal;
    return decimal;
}

// 解析 RMC 语句，成功返回 true
bool parseRMC(char* line, double* latitude, double* longitude) {
    if (strncmp(line, "$GNRMC", 6) != 0 && strncmp(line, "$GPRMC", 6) != 0) {
        return false;
    }

    char* fields[20] = {0};
    int count = 0;
    char* context = NULL;

    fields[count++] = strtok_r(line, ",", &context);
    while (count < 20 && (fields[count] = strtok_r(NULL, ",", &context)) != NULL) {
        count++;
    }

    if (count < 7) return false;
    if (fields[2] == NULL || strcmp(fields[2], "A") != 0) return false;
    if (fields[3] == NULL || fields[4] == NULL || fields[5] == NULL || fields[6] == NULL) return false;

    *latitude  = nmeaToDecimal(fields[3], fields[4][0]);
    *longitude = nmeaToDecimal(fields[5], fields[6][0]);

    return true;
}

// 非阻塞读取 GPS
// 每次调用只读取当前串口缓冲区已有数据，不等待、不delay
void updateGPSNonBlocking() {
    while (GPS_Serial.available()) {
        char c = GPS_Serial.read();

        if (c == '\n') {
            if (gpsBufferIdx > 0) {
                if (gpsLineBuffer[gpsBufferIdx - 1] == '\r') {
                    gpsLineBuffer[gpsBufferIdx - 1] = '\0';
                } else {
                    gpsLineBuffer[gpsBufferIdx] = '\0';
                }

                char lineCopy[128];
                strncpy(lineCopy, gpsLineBuffer, sizeof(lineCopy));
                lineCopy[sizeof(lineCopy) - 1] = '\0';

                double lat, lon;
                if (parseRMC(lineCopy, &lat, &lon)) {
                    gpsLat = lat;
                    gpsLon = lon;
                    gpsHasFix = true;
                    lastGpsFixTime = millis();
                }
            }

            gpsBufferIdx = 0;
            memset(gpsLineBuffer, 0, sizeof(gpsLineBuffer));
        } 
        else {
            if (gpsBufferIdx < sizeof(gpsLineBuffer) - 1) {
                gpsLineBuffer[gpsBufferIdx++] = c;
            } else {
                gpsBufferIdx = 0;
                memset(gpsLineBuffer, 0, sizeof(gpsLineBuffer));
            }
        }
    }
}

// 计算当前位置到目标点的航向角
// 返回：0=北，90=东，180=南，270=西
double bearingToTarget(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;

    double latAvgRad = radians((lat1 + lat2) / 2.0);
    double dEast  = radians(lon2 - lon1) * R * cos(latAvgRad);
    double dNorth = radians(lat2 - lat1) * R;

    double bearing = atan2(dEast, dNorth) * 180.0 / PI;

    while (bearing >= 360.0) bearing -= 360.0;
    while (bearing < 0.0) bearing += 360.0;

    return bearing;
}

// 计算 target-current 的最短角度误差
// 返回 >0：目标大致在右侧
// 返回 <0：目标大致在左侧
float angleError360(float target, float current) {
    float err = target - current;

    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;

    return err;
}

// ========================================== 电机驱动部分 ========================================== //
// ======================================= 包含水平和垂直控制 ======================================= //
const int pwmFrequency = 50;
const int pwmResolution = 14;
uint32_t usToDuty(int us) {
    return (uint32_t)((uint64_t)us * (1 << pwmResolution) / 20000);
}

// ======================== 航向控制（PID + 两个水平电机） ========================
#define MOTOR_COUNT 2
const int escPins[MOTOR_COUNT] = {41, 42};
const int ledcChannels[MOTOR_COUNT] = {0, 1};

bool yawInitialized = false;  //为否时重置直行参数

// PID 参数
float Kp = 0.8;
float Ki = 0.015;
float Kd = 0.2;
float integral = 0.0;
float prevError = 0.0;
unsigned long lastPIDTime = 0;

void horizontalSetSpeed(int motorIndex, int pulseWidthUs) {
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) return;
    ledcWrite(ledcChannels[motorIndex], usToDuty(constrain(pulseWidthUs, 1300, 1700))); // 电机脉宽范围（1300~1700µs，1500停止）
}

void setAllHorizontalMotors(int speedUs = 1500) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        horizontalSetSpeed(i, speedUs);
    }
}

float computeYawPID() {
    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0f;

    if (dt < 0.0001f) dt = 0.0001f;   // 极小值保护
    if (dt > 0.05f) dt = 0.05f;   // 限制最大间隔

    float error = targetYaw - currentYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    integral += error * dt;
    integral = constrain(integral, -20.0f, 20.0f);

    float derivative = (error - prevError) / dt;

    float output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, -20.0f, 20.0f);

    prevError = error;
    lastPIDTime = now;
    return output;
}

// ======================== 深度控制（PID + 两个垂直电机） ========================
#define DEPTH_MOTOR_COUNT 2
const int depthEscPins[DEPTH_MOTOR_COUNT] = {15, 16};
const int depthLedcChannels[DEPTH_MOTOR_COUNT] = {2, 3};

// 深度 PID 参数
float Kp_depth = 18.0;
float Ki_depth = 0.5;
float Kd_depth = 14.0;
float integral_depth = 0.0;
float prevError_depth = 0.0;
float deadzone_bias = 12;  //死区
unsigned long lastDepthPIDTime = 0;

void depthSetSpeed(int motorIndex, int pulseWidthUs) {
    if (motorIndex < 0 || motorIndex >= DEPTH_MOTOR_COUNT) return;
    ledcWrite(depthLedcChannels[motorIndex], usToDuty(constrain(pulseWidthUs, 1300, 1700))); // 电机脉宽范围（1300~1700µs，1500停止）
}

void setAllDepthMotors(int pulseWidthUs) {
    for (int i = 0; i < DEPTH_MOTOR_COUNT; i++) {
        depthSetSpeed(i, pulseWidthUs);
    }
}

// 深度 PID 计算并输出到电机（应周期性调用）
void updateDepthControl() {
    unsigned long now = millis();
    float dt = (now - lastDepthPIDTime) / 1000.0f;
    if (dt < 0.0001f) dt = 0.0001f;   // 极小值保护
    if (dt > 0.05f) dt = 0.05f;   // 限制最大间隔

    float error = targetDepth - currentDepth;
    integral_depth += error * dt;
    integral_depth = constrain(integral_depth, -20.0f, 20.0f);
    float derivative = (error - prevError_depth) / dt;
    float output = Kp_depth * error + Ki_depth * integral_depth + Kd_depth * derivative;
    output = constrain(output, -30.0f, 30.0f);
    // if (fabs(output) < deadzone_bias) {  //未到达目标深度时死区补偿，暂时不使用
    //     output = (error > 0) ? deadzone_bias : -deadzone_bias;
    // }
    prevError_depth = error;
    lastDepthPIDTime = now;
    // 将 PID 输出 (-30..30) 映射到脉宽 (1400..1600) 
    int pulse;
    if (output > 0) {
        pulse = 1531 + (int)(output * 3.333f);
    } else if (output < 0) {
        pulse = 1469 + (int)(output * 3.333f);  // output为负，加负值
    } else {
        pulse = 1500;  // 精确停转
    }
    pulse = constrain(pulse, 1360, 1640);
    setAllDepthMotors(pulse);
}

/**
 * 原地旋转指定角度（阻塞式）
 * 
 * 参数：
 *   targetAngleDelta - 目标转动角度（度），正值表示逆时针，负值表示顺时针。默认 +90°。
 *   turnSpeedUs       - 转弯时的基础脉宽（µs），默认 1600。数值越大转速越快。
 *   timeoutMs         - 超时时间（毫秒），防止因传感器故障导致死循环。默认 5000ms。
 * 
 * 返回值：
 *   true  - 成功转到目标角度
 *   false - 超时或传感器数据无效
 * 
 * 注意：
 *   - 转弯过程中会暂停直行 PID 控制，转弯结束后自动将转弯的目标航向设为新的直行目标。
 *   - 阻塞执行，转弯期间无法响应其他指令。
 */
static inline float normalizeAngle(float angle) {
    angle = fmodf(angle, 360.0f); 
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

bool turnByAngleUnderwater(float targetAngleDelta = 90.0, int turnSpeedUs = 1600, unsigned long timeoutMs = 8000) {
    
    float startYaw = targetYaw;
    
    float targetNormalized = normalizeAngle(startYaw + targetAngleDelta);
    targetYaw = targetNormalized;
    const int MIN_EFFECTIVE_US = 1480;  // 放过冲下限，可根据实际测试调整
    
    unsigned long startTime = millis();
    bool success = false;
    
    while (millis() - startTime < timeoutMs) {
        updateYawReading();
        
        float currentNormalized = normalizeAngle(currentYaw);
        float diff = currentNormalized - targetNormalized;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        
        float remaining = fabs(diff);
        
        // 到达目标
        if (remaining < 10.0f) {
            success = true;
            break;
        }
        
        // 计算当前应使用的转弯速度
        int speedUs = turnSpeedUs;
        
        // 接近目标时适当降速，防过冲
        if (remaining < 50.0f) {
            speedUs = map(remaining, 0, 50, MIN_EFFECTIVE_US, turnSpeedUs);
        }
        
        // 速度过低时直接退出
        if (1480 < speedUs && speedUs < 1520) {
            break;
        }

        // 设置差速转向
        int leftPulse, rightPulse;
        if (targetAngleDelta > 0) {
            leftPulse  = speedUs;
            rightPulse = 1500;
        } else {
            leftPulse  = 1500;
            rightPulse = speedUs;
        }
        
        leftPulse  = constrain(leftPulse,  1300, 1700);
        rightPulse = constrain(rightPulse, 1300, 1700);
        
        horizontalSetSpeed(0, leftPulse);
        horizontalSetSpeed(1, rightPulse);
        updateDepthReading();      // 读取最新深度
        updateDepthControl();      // 计算并输出深度电机 PWM,函数内不带深度读取
        delay(5);
    }
    
    setAllHorizontalMotors(1500);
    delay(500);  // 短暂延时
    
    yawInitialized = false;
    return success;
}

void escCalibrate() {  //电机校准
    setAllHorizontalMotors(2000);
    setAllDepthMotors(2000);
    delay(2000);
    setAllHorizontalMotors(1500);
    setAllDepthMotors(1500);
    delay(2000);
}

void setupUnusedPins() {
    const int availablePins[] = {
        1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,
        38,39,40,41,42,43,44,47,48
    };
    const int skipPins[] = {41,42,40,39,1,2,15,16,13,14};  //此处输入使用引脚
    for (unsigned int i = 0; i < sizeof(availablePins)/sizeof(availablePins[0]); i++) {
        int pin = availablePins[i];
        bool shouldSkip = false;
        for (unsigned int j = 0; j < sizeof(skipPins)/sizeof(skipPins[0]); j++) {
            if (pin == skipPins[j]) {
                shouldSkip = true;
                break;
            }
        }
        if (!shouldSkip) {
            pinMode(pin, INPUT_PULLUP);
            delay(1);
        }
    }
}

bool readGpsOnce(double& lat, double& lon, unsigned long timeoutMs) {
    unsigned long startTime = millis();

    while (millis() - startTime < timeoutMs) {
        updateGPSNonBlocking();

        if (gpsHasFix) {
            lat = gpsLat;
            lon = gpsLon;
            return true;
        }

        delay(10);
    }

    return false;
}

double startLat1 = 0.0;
double startLon1 = 0.0;
double startLat2 = 0.0;
double startLon2 = 0.0;

bool hasStartPoint1 = false;
bool hasStartPoint2 = false;

float startMoveHeading = 0.0f;      // GPS算出来的机器人初始前进方向
float imuForwardOffset = 0.0f;      // IMU航向 - 机器人真实前进方向
bool imuForwardCalibrated = false;

float normalize360Float(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

void setup() {
    delay(200);
    setupUnusedPins();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledcSetup(ledcChannels[i], pwmFrequency, pwmResolution);
        ledcAttachPin(escPins[i], ledcChannels[i]);
    }
    for (int i = 0; i < DEPTH_MOTOR_COUNT; i++) {
        ledcSetup(depthLedcChannels[i], pwmFrequency, pwmResolution);
        ledcAttachPin(depthEscPins[i], depthLedcChannels[i]);
    }
    escCalibrate();  //电机初始化
    pinMode(RX_PIN, INPUT_PULLUP);
    Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);  //深度初始化
    initCH0X0();  //陀螺仪初始化
    
    delay(15000);  // 等待传感器输出稳定
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);  // GPS初始化
    // 先读一次起点
    hasStartPoint1 = readGpsOnce(startLat1, startLon1, 20000);

    // 这里的5秒内，机器人要向前走一小段
    unsigned long waitStart = millis();
    while (millis() - waitStart < 5000) {
        updateGPSNonBlocking();
        updateYawReading();

        // 这里如果你要让机器人自己往前走，可以给水平电机一点速度
        // 不想自动走，就人工推/拖动也行
        // setAllHorizontalMotors(1540);

        delay(10);
    }

    setAllHorizontalMotors(1500);

    hasStartPoint2 = readGpsOnce(startLat2, startLon2, 10000);

    if (hasStartPoint1 && hasStartPoint2) {
        startMoveHeading = (float)bearingToTarget(
            startLat1, startLon1,
            startLat2, startLon2
        );

        updateYawReading();

        imuForwardOffset = angleError360(currentYaw, startMoveHeading);
        imuForwardCalibrated = true;
    }
}

void loop() {
    updateGPSNonBlocking();
    updateYawReading();

    if (!gpsHasFix || !imuForwardCalibrated) {
        setAllHorizontalMotors(1500);
        return;
    }

    double targetHeading = bearingToTarget(gpsLat, gpsLon, TARGET_LAT, TARGET_LON);

    float robotYaw = normalize360Float(currentYaw - imuForwardOffset);
    float error = angleError360((float)targetHeading, robotYaw);

    float absError = fabs(error);

    int speedUs = map((int)absError, 0, 180, 1530, 1650);
    speedUs = constrain(speedUs, 1530, 1650);

    if (absError < 10) {
        setAllHorizontalMotors(1530);
    } 
    else if (error > 0) {
        horizontalSetSpeed(0, speedUs);
        horizontalSetSpeed(1, 1500);
    } 
    else {
        horizontalSetSpeed(0, 1500);
        horizontalSetSpeed(1, speedUs);
    }
}
