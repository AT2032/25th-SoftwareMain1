//Analog UARTの宣言
#define SerialAir Serial1 //AirのUART
#define SerialUnder Serial2 //UnderのUART

//PIO UARTの宣言 
SerialPIO SerialGPS(2, 3); //GPSのUART
SerialPIO SerialSupport_ICS(10, 11); //2層目とICSのUART

//GPS
#include <TinyGPSPlus.h>
TinyGPSPlus gps;

//接続確認用
unsigned long lastReceivedTime = 0; // 最後に有効なデータを受信した時間
const unsigned long connectionTimeout = 5000; // 接続タイムアウト（ミリ秒）．とりあえず5秒とした．もっと長くしてもいいような気もする

//SD送信用バッファ
char UART_SD[512];

//TORICA_UARTインスタンス化
#include <TORICA_UART.h>
TORICA_UART Under_UART(&SerialUnder);
TORICA_UART Air_UART(&SerialAir);
TORICA_UART Support_ICS_UART(&SerialSupport_ICS);

//動作確認用LED
const int L_Air = 18;
const int L_Under = 19;
const int L_ICS = 20;

//ICS初期化
#include <TORICA_ICS.h>
TORICA_ICS ics(&SerialSupport_ICS);

//I2Cパッケージ読み込み
#include <Wire.h>

//BNO055インスタンス化(あとでGeometry.hを使ったプログラムを考える)
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <utility/Matrix.h>
#include <utility/quaternion.h>
#include <utility/vector.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire1);
struct EulerAngles {
  float roll, pitch, yaw;
};
EulerAngles toEulerAngles(float qx, float qy, float qz, float qw) {
  EulerAngles angles;

  // ロール角 (X軸周りの回転)
  float sinr_cosp = 2 * (qw * qx + qy * qz);
  float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  angles.roll = atan2(sinr_cosp, cosr_cosp);

  // ピッチ角 (Y軸周りの回転)
  float sinp = 2 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    angles.pitch = copysign(M_PI / 2, sinp); // ピッチが±90度の場合
  else
    angles.pitch = asin(sinp);

  // ヨー角 (Z軸周りの回転)
  float siny_cosp = 2 * (qw * qz + qx * qy);
  float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  angles.yaw = atan2(siny_cosp, cosy_cosp);

  return angles;
}

//DPS310初期化
#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

//Flight-Phaseの定義
enum {
  PLATFORM,
  HIGH_LEVEL,
  MID_LEVEL,
  LOW_LEVEL
} flight_phase = PLATFORM;//おそらく"PLATFORM"で初期化

bool TAKEOFF = false;

enum {
  FAST,
  NORMAL,
  SLOW,
} speed_level = NORMAL;

// filtered:移動平均
// lake:対地高度, 無印:気圧基準海抜高度
// dps:気圧高度
// urm:超音波高度

const float const_platform_m = 10.6; //プラットフォームの高さ+機体の高さ

#include <TORICA_MoveAve.h>
// 対気速度
TORICA_MoveAve<5> filtered_airspeed_ms(0);

// 現在の気圧高度(気圧基準)
TORICA_MoveAve<5> filtered_main_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_under_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_air_dps_altitude_m(0);
// プラホの高度(気圧基準)
TORICA_MoveAve<50> main_dps_altitude_platform_m(0);
TORICA_MoveAve<50> under_dps_altitude_platform_m(0);
TORICA_MoveAve<50> air_dps_altitude_platform_m(0);

// xy軸の加速度の平均値
TORICA_MoveAve<3> filtered_main_bno_accx_mss(0); //勘
TORICA_MoveAve<3> filtered_main_bno_accy_mss(0); //勘

// 気圧センサを用いた信頼できる対地高度
// 3つの気圧高度にそれぞれ移動平均をとってプラホを10mとし，中央値をとった値
#include <QuickStats.h>
float dps_altitude_lake_array_m[3];
QuickStats dps_altitude_lake_m;

// 超音波高度(対地高度)
TORICA_MoveAve<3> filtered_under_urm_altitude_m(0);

#include<TORICA_MoveMedian.h>
// 気圧での対地高度と超音波での対地高度の差
// 100Hz(calculate)*4s = 400
TORICA_MoveMedian<400> altitude_dps_urm_offset_m(0);


// 気圧と超音波から推定した対地高度
float estimated_altitude_lake_m = const_platform_m;

//エアデータと機体下電装部の生存確認(UART通信で他の基板に送るため，volatileで定義しないといけない可能性がある)
bool air_is_alive = false;
bool under_is_alive = false;

// ---- sensor data value  ----
//    data_マイコン名_センサー名_データ種類_単位
//main電装
//BNO055
volatile float data_main_bno_accx_mss = 0;
volatile float data_main_bno_accy_mss = 0;
volatile float data_main_bno_accz_mss = 0;
volatile float data_main_bno_qw = 0;
volatile float data_main_bno_qx = 0;
volatile float data_main_bno_qy = 0;
volatile float data_main_bno_qz = 0;
volatile float data_main_bno_roll = 0;
volatile float data_main_bno_pitch = 0;
volatile float data_main_bno_yaw = 0;
//DPS310
volatile float data_main_dps_pressure_hPa = 0;
volatile float data_main_dps_temperature_deg = 0;
volatile float data_main_dps_altitude_m = 0;
//GPS
volatile uint8_t data_main_gps_hour = 0;
volatile uint8_t data_main_gps_minute = 0;
volatile uint8_t data_main_gps_second = 0;
volatile uint8_t data_main_gps_centisecond = 0;
volatile double data_main_gps_latitude_deg = 0;
volatile double data_main_gps_longitude_deg = 0;
volatile double data_main_gps_altitude_m = 0;

//Under電装部
volatile float data_under_dps_pressure_hPa = 0;
volatile float data_under_dps_temperature_deg = 0;
volatile float data_under_dps_altitude_m = 0;
volatile float data_under_urm_altitude_m = 0;

//Airdata電装部
volatile float data_air_dps_pressure_hPa = 0;
volatile float data_air_dps_temperature_deg = 0;
volatile float data_air_dps_altitude_m = 0;
volatile float data_air_sdp_differentialPressure_Pa = 0;
volatile float data_air_sdp_airspeed_ms = 0;
volatile float data_air_AoA_angle_deg = 0;
volatile float data_air_AoS_angle_deg = 0;
volatile float data_air_bno_accx_mss = 0;
volatile float data_air_bno_accy_mss = 0;
volatile float data_air_bno_accz_mss = 0;
volatile float data_air_bno_qw = 0;
volatile float data_air_bno_qx = 0;
volatile float data_air_bno_qy = 0;
volatile float data_air_bno_qz = 0;
volatile float data_air_bno_roll = 0;
volatile float data_air_bno_pitch = 0;
volatile float data_air_bno_yaw = 0;
//ICS基盤
volatile int data_ics_angle = 0;

// ----------------------------

void setup() {
  //LED初期化
  pinMode(L_ICS, OUTPUT);
  pinMode(L_Under, OUTPUT);
  pinMode(L_Air, OUTPUT);

  // USBケーブルを差した時の起動猶予時間
  for (int i = 0; i < 3; i++) {
    digitalWrite(L_ICS, HIGH);
    digitalWrite(L_Under, HIGH);
    digitalWrite(L_Air, HIGH);
    delay(400);
    digitalWrite(L_ICS, LOW);
    digitalWrite(L_Under, LOW);
    digitalWrite(L_Air, LOW);
    delay(100);
  }

  delay(2000);

  //UARTピン設定
  SerialUnder.setTX(4);
  SerialUnder.setRX(5);
  SerialAir.setTX(0);
  SerialAir.setRX(1);

  // UART初期化
  //SerialGPS.setFIFOSize(1024);
  SerialAir.setFIFOSize(1024);
  SerialUnder.setFIFOSize(1024);
  //SerialSupport_ICS.setFIFOSize(1024);

  //通信速度設定
  SerialGPS.begin(460800);  //GPS
  SerialSupport_ICS.begin(115200);  //SDマイコン+ICS
  SerialAir.begin(460800);
  SerialUnder.begin(460800);
  Serial.begin(115200);

  //I2C wire1ピン設定
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  //DPS310初期化
  Wire1.setClock(400000);
  //DPS310と通信できているかの判定 DPS310が通信できていない場合はL_Airが点滅する
  if (!dps.begin_I2C(0x77, &Wire1)) {
    Serial.println("Failed to find DPS");
    while (1) {
      //SerialWireless.println("Failed to find DPS");
      digitalWrite(L_Air, HIGH);
      delay(100);
      digitalWrite(L_Air, LOW);
      delay(100);
    }
  }
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);
  Serial.println("DPS OK!");

  // BNO055初期化　BNO055が通信できていない場合はL_Underが点滅する
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {
      digitalWrite(L_Under, HIGH);
      delay(100);
      digitalWrite(L_Under, LOW);
      delay(100);
    }
  }

    // センサー・各基板の起動を待機
  for (int i = 0; i < 3; i++) {
    digitalWrite(L_ICS, HIGH);
    digitalWrite(L_Under, HIGH);
    digitalWrite(L_Air, HIGH);
    delay(100);
    digitalWrite(L_ICS, LOW);
    digitalWrite(L_Under, LOW);
    digitalWrite(L_Air, LOW);
    delay(400);
  }
}

void loop() {
  uint32_t ISR_now_time = millis(); //100Hzで測定
  static uint32_t ISR_last_time = 0;
  if (ISR_now_time - ISR_last_time >= 10) {
    ISR_last_time = millis();
    func_100Hz();
  }
}

//各関数の実行
void func_100Hz() {
  //uint32_t time_us = micros();

  // GPS・操舵角・機体下電装部・エアデータ電装部読み取り
  polling_UART();

  // フライトフェーズ判断(メインの加速度も測定)
  determine_flight_phase();

  // 気圧センサ×3・超音波センサから高度推定
  calculate_altitude();

  // SDに記録(メインの気圧高度も測定)
  send_SD();

  // if (micros() - time_us > 9900) {  //MAX10000=100Hz
  //   Serial.print("ISR100Hz_overrun!!!");
  // }
  // Serial.print("ISR_us:");
  // Serial.println(micros() - time_us);

}

void polling_UART() {
  //ICS
  data_ics_angle = ics.read_Angle();
  if (data_ics_angle > 0) {
    digitalWrite(L_ICS, !digitalRead(L_ICS));
  }

  //UnderSide
  static unsigned long int last_under_time_ms = 0;
  int readnum = Under_UART.readUART();
  int under_data_num = 4;
  if (readnum == under_data_num) {
    last_under_time_ms = millis();
    digitalWrite(L_Under, !digitalRead(L_Under));
    data_under_dps_pressure_hPa = Under_UART.UART_data[0];
    data_under_dps_temperature_deg = Under_UART.UART_data[1];
    data_under_dps_altitude_m = Under_UART.UART_data[2];
    data_under_urm_altitude_m = Under_UART.UART_data[3];
    filtered_under_dps_altitude_m.add(data_under_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      under_dps_altitude_platform_m.add(data_under_dps_altitude_m);
    }
    filtered_under_urm_altitude_m.add(data_under_urm_altitude_m);
  }
  if (millis() - last_under_time_ms > 1000) {
    // 超音波高度のみ冗長系がないため，データが来なければ8mとして高度推定に渡す．
    // 測定範囲外のときは10mになり，9m以上でテイクオフ判断をするため故障時は8m
    // filtered_under_urm_altitude_m.add(8.0);
    // ToDo 明示的にis_aliveを作るべき．値の処理によって7変わる．
    under_is_alive = false;
  } else {
    under_is_alive = true;
  }

  //AirData
  static unsigned long int last_air_time_ms = 0;
  readnum = Air_UART.readUART();
  int air_data_num = 17;
  if (readnum == air_data_num) {
    last_air_time_ms = millis();
    digitalWrite(L_Air, !digitalRead(L_Air));
    data_air_dps_pressure_hPa = Air_UART.UART_data[0];
    data_air_dps_temperature_deg = Air_UART.UART_data[1];
    data_air_dps_altitude_m = Air_UART.UART_data[2];
    data_air_sdp_differentialPressure_Pa = Air_UART.UART_data[3];
    data_air_sdp_airspeed_ms = Air_UART.UART_data[4];
    data_air_AoA_angle_deg = Air_UART.UART_data[5];
    data_air_AoS_angle_deg = Air_UART.UART_data[6];
    data_air_bno_accx_mss = Air_UART.UART_data[7];
    data_air_bno_accy_mss = Air_UART.UART_data[8];
    data_air_bno_accz_mss = Air_UART.UART_data[9];
    data_air_bno_qw = Air_UART.UART_data[10];
    data_air_bno_qx = Air_UART.UART_data[11];
    data_air_bno_qy = Air_UART.UART_data[12];
    data_air_bno_qz = Air_UART.UART_data[13];
    data_air_bno_roll = Air_UART.UART_data[14];
    data_air_bno_pitch = Air_UART.UART_data[15];
    data_air_bno_yaw = Air_UART.UART_data[16];
    filtered_airspeed_ms.add(data_air_sdp_airspeed_ms);
    filtered_air_dps_altitude_m.add(data_air_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      air_dps_altitude_platform_m.add(data_air_dps_altitude_m);
    }
  }
   if (millis() - last_air_time_ms > 1000) {
    air_is_alive = false;
  } else {
    air_is_alive = true;
  }

  //GPS
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      data_main_gps_hour = gps.time.hour();
      data_main_gps_minute = gps.time.minute();
      data_main_gps_second = gps.time.second();
      data_main_gps_centisecond = gps.time.centisecond();
      data_main_gps_latitude_deg = gps.location.lat();
      data_main_gps_longitude_deg = gps.location.lng();
      data_main_gps_altitude_m = gps.altitude.meters();
    }
  }
}

void determine_flight_phase() {
  //発進判定のため，IMU測定はここで行う
  read_main_bno();

  static unsigned long int takeoff_time_ms = 0;
  switch (flight_phase) {
    case PLATFORM:
      {
        static int over_urm_range_count = 0;
        if (filtered_under_urm_altitude_m.get() > 9.0) {
          over_urm_range_count++;
        } else {
          over_urm_range_count = 0;
        }
        bool over_urm_range = false;
        // 超音波が測定不能な状態が2秒以上続いたとき
        if (over_urm_range_count >= 100) {
          over_urm_range = true;
        }
        // 気圧センサにより下降したと判断したとき
        bool descending = estimated_altitude_lake_m < 10.2;
        //x軸またはy軸方向に急激に加速した場合(値はセンサの向きのよって変える)
        bool x_accelerated = abs(filtered_main_bno_accx_mss.get()) > 3.0;
        bool y_accelerated = abs(filtered_main_bno_accy_mss.get()) > 3.0;
        // x軸方向の加速度またはy軸方向の急激な加速と超音波センサと気圧センサによる下降判断そしてマイコンが起動してから10秒後
        if ((over_urm_range || descending) && (x_accelerated || y_accelerated) && millis() > 10000){
          TAKEOFF = true;
          flight_phase = HIGH_LEVEL;
        }
      }
      break;
    case HIGH_LEVEL:
      if ( 0.3 <= filtered_under_urm_altitude_m.get() < 1.0){
        flight_phase = MID_LEVEL;
      }
      else if ( filtered_under_urm_altitude_m.get() < 0.3){
        flight_phase = LOW_LEVEL;
      }
      else{
        flight_phase = HIGH_LEVEL;
      }
      break;
    case MID_LEVEL:
      if ( 0.3 <= filtered_under_urm_altitude_m.get() < 1.0){
        flight_phase = MID_LEVEL;
      }
      else if ( filtered_under_urm_altitude_m.get() < 0.3){
        flight_phase = LOW_LEVEL;
      }
      else{
        flight_phase = HIGH_LEVEL;
      }
      break;
    case LOW_LEVEL:
      if ( 0.3 <= filtered_under_urm_altitude_m.get() < 1.0){
        flight_phase = MID_LEVEL;
      }
      else if ( filtered_under_urm_altitude_m.get() < 0.3){
        flight_phase = LOW_LEVEL;
      }
      else{
        flight_phase = HIGH_LEVEL;
      }
      break;
    default:
      break;
  }
  //速度レベル判断
  if (filtered_airspeed_ms.get() > 1.5) {
    speed_level = FAST;
  }
  else if (filtered_airspeed_ms.get() > 1.0) {
    speed_level = NORMAL;
  }
  else {
    speed_level = SLOW;
  }
}

void calculate_altitude() {
  // 100Hzで関数呼び出し
  read_main_dps();

  dps_altitude_lake_array_m[0] = filtered_main_dps_altitude_m.get() - main_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[1] = filtered_under_dps_altitude_m.get() - under_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[2] = filtered_air_dps_altitude_m.get() - air_dps_altitude_platform_m.get() + const_platform_m;
  estimated_altitude_lake_m = dps_altitude_lake_m.median(dps_altitude_lake_array_m, 3);

  // 関数は100Hzで呼び出される
  // 中央値が出始めるのに2秒，そこから3秒間で気圧から超音波に情報源を切り替え
  static int transtion_count = 0;
  if ( filtered_under_urm_altitude_m.get() < 8.0) {
    // 気圧センサが本来より低い値ならオフセットは正
    altitude_dps_urm_offset_m.add(filtered_under_urm_altitude_m.get() - estimated_altitude_lake_m );

    if (transtion_count < 500) {
      transtion_count++;
    }
    float ratio = 1;
    if (transtion_count < 500) {
      ratio = 0;
    }
    if (transtion_count > 200) {
      ratio = (float)(transtion_count - 200) / 300.0;
    }
    // 気圧センサが本来より低い値なら正のオフセットを足す
    estimated_altitude_lake_m += altitude_dps_urm_offset_m.get() * ratio;

  }
}

void send_SD() {
  uint32_t time_ms = millis();
  static int loop_count = 0;
  if (loop_count == 0) {
    sprintf(UART_SD, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",  //11個
            time_ms, data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss, 
            data_main_bno_qw, data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, 
            data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
  } else if (loop_count == 1) {
    sprintf(UART_SD, "%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", //11個
            estimated_altitude_lake_m, altitude_dps_urm_offset_m.get(), flight_phase, speed_level,
            data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m, data_under_dps_pressure_hPa,
            data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m);
  } else if (loop_count == 2) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,",  //8個
            data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m,
            data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms, data_air_AoA_angle_deg,
            data_air_AoS_angle_deg, data_ics_angle);
  } else if (loop_count == 3) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",   //10個
            data_air_bno_accx_mss, data_air_bno_accy_mss, data_air_bno_accz_mss,
            data_air_bno_qw, data_air_bno_qx, data_air_bno_qy, data_air_bno_qz,
            data_air_bno_roll, data_air_bno_pitch, data_air_bno_yaw);
  } else {
    sprintf(UART_SD, "%u,%u,%u.%u,%10.7lf,%10.7lf,%5.2lf,%d,%d\n",    //9個
            data_main_gps_hour, data_main_gps_minute, data_main_gps_second, data_main_gps_centisecond,
            data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m, under_is_alive, air_is_alive);
    loop_count = -1;
  }
  loop_count++;
  //バッファをクリアしてから新しいデータを書き込み
  SerialSupport_ICS.flush();
  SerialAir.flush();
  SerialUnder.flush();
  SerialSupport_ICS.print(UART_SD);
  SerialAir.print(UART_SD);
  SerialUnder.print(UART_SD);
}

void read_main_bno() {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); //加速度ベクトルを取得（センサーの生の値）
  imu::Quaternion quat = bno.getQuat(); //クォータニオンを取得（センサーの生の値）
  data_main_bno_accx_mss = accel.x(); //x方向の加速度を格納
  data_main_bno_accy_mss = accel.y(); //y方向の加速度を格納
  data_main_bno_accz_mss = accel.z(); //z方向の加速度を格納
  data_main_bno_qw = quat.w(); //クォータニオン(回転量w)を格納
  data_main_bno_qx = quat.x(); //クォータニオン(x軸)を格納
  data_main_bno_qy = quat.y(); //クォータニオン(y軸)を格納
  data_main_bno_qz = quat.z(); //クォータニオン(z軸)を格納
  EulerAngles euler = toEulerAngles(data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, data_main_bno_qw);
  data_main_bno_roll = euler.roll * 180 / M_PI;   // ラジアンを度に変換
  data_main_bno_pitch = euler.pitch * 180 / M_PI; // ラジアンを度に変換
  data_main_bno_yaw = euler.yaw * 180 / M_PI;     // ラジアンを度に変換

  /*参考
  https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf
  */
}

void read_main_dps() {
  if (!(dps.temperatureAvailable() && dps.pressureAvailable())) {
    return;
  }
  dps.getEvents(&temp_event, &pressure_event);
  data_main_dps_pressure_hPa = pressure_event.pressure;
  data_main_dps_temperature_deg = temp_event.temperature;
  data_main_dps_altitude_m = (powf(1013.25 / data_main_dps_pressure_hPa, 1 / 5.257) - 1) * (data_main_dps_temperature_deg + 273.15) / 0.0065;
  filtered_main_dps_altitude_m.add(data_main_dps_altitude_m);
  if (flight_phase == PLATFORM) {
    main_dps_altitude_platform_m.add(data_main_dps_altitude_m);
  }
}

