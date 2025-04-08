#系統運行v4
import network
import socket
from machine import Pin, PWM, ADC, time_pulse_us
import time
import utime
import _thread

# WiFi 設定【根據不同環境設定自己的WiFi】
SSID = "shaneiPhone"
PASSWORD = "299792458"
PORT = 12345

# 設定所有 PWM 腳位
servo_pin = Pin(18)  # 控制底座伺服
servo_pin2 = Pin(21)  # 控制大臂伺服
servo_pin3 = Pin(22)  # 控制爪子伺服
servo_pin4 = Pin(17)  # 控制小臂伺服
servo_pin7 = Pin(15)  # 分類圓盤

# 初始化所有 PWM
pwm = PWM(servo_pin, freq=50)
pwm2 = PWM(servo_pin2, freq=50)
pwm3 = PWM(servo_pin3, freq=50)
pwm4 = PWM(servo_pin4, freq=50)
pwm7 = PWM(servo_pin7, freq=50)

#====<包裝設定>================================================
servo_pin5_Package = Pin(19)  # 控制紅色包裝伺服 
pwm5_Package = PWM(servo_pin5_Package, freq=50)
servo_pin6_Package = Pin(27)  # 控制綠色包裝伺服 
pwm6_Package = PWM(servo_pin6_Package, freq=50)

# 設定光敏電阻腳位 - 綠色包裝系統
green_ldr_pin1 = ADC(Pin(34))      # 光敏電阻接 GPIO34
green_ldr_pin2 = ADC(Pin(33))      # 光敏電阻接 GPIO33
green_ldr_pin3 = ADC(Pin(32))      # 光敏電阻接 GPIO32

# 設定光敏電阻腳位 - 紅色包裝系統
red_ldr_pin1 = ADC(Pin(35))      # 光敏電阻接 GPIO35(14)
red_ldr_pin2 = ADC(Pin(36))      # 光敏電阻接 GPIO36(13)
red_ldr_pin3 = ADC(Pin(39))      # 光敏電阻接 GPIO39(12)

# 設定 ADC - 綠色包裝系統
green_ldr_pin1.atten(ADC.ATTN_11DB)    
green_ldr_pin2.atten(ADC.ATTN_11DB)
green_ldr_pin3.atten(ADC.ATTN_11DB)

# 設定 ADC - 紅色包裝系統
red_ldr_pin1.atten(ADC.ATTN_11DB)    
red_ldr_pin2.atten(ADC.ATTN_11DB)
red_ldr_pin3.atten(ADC.ATTN_11DB)

# 定義伺服馬達角度對應的duty值範圍
MIN_DUTY_Package = 40
MAX_DUTY_Package = 115
#=============================================================

# 輸送帶設定
relay_pin = Pin(5, Pin.OUT)
trigger_pin = Pin(2, Pin.OUT)
echo_pin = Pin(4, Pin.IN)

#====<全域範圍變數宣告>==================
# 線程控制變數
current_distance = None
stop_threads = False
conveyor_control = 's'  # 輸送帶控制狀態
lock = _thread.allocate_lock()
# Socket
s = None  # 用於存儲 socket 
# 全局變量來存儲包裝讀取初始平均值
green_avg1, green_avg2, green_avg3 = None, None, None
red_avg1, red_avg2, red_avg3 = None, None, None
# 控制包裝系統線程運行的標誌
package_thread_running = False
#=======================================

#====<定義函式區>====#
def connect_wifi():
    """連接到 WiFi 網路並初始化 Socket 伺服器"""
    global s  
    
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    #設定固定IP
    wlan.ifconfig(('', '255.255.255.0', '', '8.8.8.8'))#ESP32IP以及相關設定，在CMD敲ipconfig取得資訊
    if not wlan.isconnected():
        print('正在連接 WiFi...')
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            pass
    print('WiFi 已連接')
    print('IP:', wlan.ifconfig()[0])
    
    # 初始化 Socket 伺服器
    addr = socket.getaddrinfo('0.0.0.0', PORT)[0][-1]
    s = socket.socket()
    s.bind(addr)
    s.listen(1)
    print('Socket 伺服器已初始化，等待連接...')

def set_servo_angle(servo, angle, delay=20, increments=50):
    """控制伺服馬達角度的通用函數"""
    min_duty = 40
    max_duty = 115
    start_duty = servo.duty()
    end_duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    step = (end_duty - start_duty) / increments
    
    for i in range(increments):
        new_duty = int(start_duty + i * step)
        servo.duty(new_duty)
        utime.sleep_ms(delay)

def set_continuous_speed(speed):
    """設置分類圓盤的連續旋轉速度"""
    if speed == 0:
        duty = 73
    else:
        if speed > 0:
            duty = int(73 + (speed / 100) * 30)
        else:
            duty = int(73 + (speed / 100) * 30)
    pwm7.duty(duty)

def arm_calibration():
    """ARM初始校正"""
    print("執行ARM校正")

    #開始動作=================
    print("開始動作")
    print("第一次")
    set_servo_angle(pwm, 90, delay=100, increments=50)#底座
    utime.sleep_ms(1000)
    print("校正底座完成")
    set_servo_angle(pwm2, 210, delay=50, increments=50)#大臂
    utime.sleep_ms(1000)
    print("校正大臂完成")
    set_servo_angle(pwm4, 180, delay=50, increments=50)#小臂
    utime.sleep_ms(1000)
    print("校正小臂完成")
    set_servo_angle(pwm3, 60, delay=100, increments=50)#爪子
    utime.sleep_ms(1000)
    print("校正爪子完成")
    print("第二次")
    set_servo_angle(pwm, 42, delay=100, increments=50)#底座
    utime.sleep_ms(1000)
    print("校正底座完成")
    set_servo_angle(pwm2, 220, delay=50, increments=50)#大臂
    utime.sleep_ms(1000)
    print("校正大臂完成")
    set_servo_angle(pwm4, 170, delay=50, increments=50)#小臂
    utime.sleep_ms(1000)
    print("校正小臂完成")
    set_servo_angle(pwm3, 200, delay=100, increments=50)#爪子
    utime.sleep_ms(1000)
    print("校正爪子完成")
    #========================

def arm1_action():
    """ARM1 動作序列：從輸送帶到圓盤"""
    print("執行 ARM1 動作")    
    #開始動作=================
    print("開始動作")
    # 底座 110度
    print("底座轉到 110 度") 
    set_servo_angle(pwm, 110, delay=100, increments=50)

    # 小臂上抬、大臂下壓 
    print("小臂上抬")
    set_servo_angle(pwm4, 150, delay=50, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 180, delay=50, increments=50)
       
    #爪子抓取===================
    print("爪子抓取")
    # 爪子張開
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=60, increments=50)
    utime.sleep_ms(1000)
    #==========================

    # 小臂下壓、大臂下壓
    print("小臂下壓")
    set_servo_angle(pwm4, 154, delay=20, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 176, delay=30, increments=50)
    print("小臂下壓")
    set_servo_angle(pwm4, 160, delay=20, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 170, delay=30, increments=50)
    print("小臂下壓")
    set_servo_angle(pwm4, 164, delay=20, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 156, delay=30, increments=50)
    print("小臂下壓")
    set_servo_angle(pwm4, 170, delay=20, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 154, delay=30, increments=50)
    set_servo_angle(pwm2, 152, delay=30, increments=50)
    set_servo_angle(pwm2, 150, delay=30, increments=50)

    # 爪子閉合
    print("爪子閉合") 
    set_servo_angle(pwm3, 120, delay=100, increments=50)       
    utime.sleep_ms(1000)
    #=========================
        
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 190, delay=100, increments=50)
    
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 160, delay=100, increments=50)
    #=========================
    
    # 底座移動到圓盤=================================        
    # 底座 20度
    print("底座移動到圓盤")
    set_servo_angle(pwm, 20
                    , delay=100, increments=50)
    print("20")
    #===============================================

    #放到辨識圓盤====================
    print("開始放到辨識圓盤")        
    #大臂下壓
    print("大臂下壓")
    set_servo_angle(pwm2, 170, delay=100, increments=50)
    set_servo_angle(pwm2, 165, delay=50, increments=50)
    set_servo_angle(pwm2, 160, delay=50, increments=50)
    set_servo_angle(pwm2, 155, delay=50, increments=50)
    set_servo_angle(pwm2, 150, delay=50, increments=50)
    set_servo_angle(pwm2, 145, delay=50, increments=50)
    set_servo_angle(pwm2, 140, delay=50, increments=50)
    set_servo_angle(pwm2, 135, delay=50, increments=50)
    set_servo_angle(pwm2, 130, delay=50, increments=50)
    set_servo_angle(pwm2, 125, delay=50, increments=50)
    set_servo_angle(pwm2, 120, delay=50, increments=50) # 注意角度
    
    # 爪子張開
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=100, increments=50)
        
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 160, delay=50, increments=50)
    
    # 爪子回正
    print("爪子回正") 
    set_servo_angle(pwm3, 200, delay=100, increments=50)
    
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 220, delay=30, increments=50)
    
    # 小臂回開始位 
    print("小臂回開始位")
    set_servo_angle(pwm4, 170, delay=30, increments=50)    
    utime.sleep_ms(1000)
    #=========================

def arm2_action():
    """ARM2 動作序列：從圓盤到紅色包裝"""
    print("執行 ARM2 動作")
    #開始動作=================
    print("開始動作")     
    # 底座 20度
    print("底座移動到圓盤")
    set_servo_angle(pwm, 20, delay=100, increments=50)
    #===============================================

    #到辨識圓盤夾取====================
    print("到辨識圓盤夾取")
    # 爪子張開===================
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=100, increments=50)
    #==========================
    
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 160, delay=20, increments=50)
    
    #大臂下壓
    print("大臂下壓")
    set_servo_angle(pwm2, 130, delay=50, increments=50)
    set_servo_angle(pwm2, 120, delay=40, increments=50)
    set_servo_angle(pwm2, 110, delay=40, increments=50)
    
     #爪子抓取===================
    print("爪子抓取")
    # 爪子閉合
    print("爪子閉合") 
    set_servo_angle(pwm3, 120, delay=100, increments=50)
        
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 190, delay=70, increments=50)
    utime.sleep_ms(500)
    
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 130, delay=20, increments=50)
    
    #轉到紅色包裝=========================
    print("轉到紅色包裝")
    # 底座 195度
    print("底座轉到 195 度") 
    set_servo_angle(pwm, 195, delay=100, increments=50)   
    utime.sleep_ms(2000)
    #====================================

    # 大臂下壓 
    #print("大臂下壓")
    set_servo_angle(pwm2, 178, delay=50, increments=50)
    
    # 小臂下壓
    print("小臂下壓")
    set_servo_angle(pwm4, 160, delay=40, increments=50)      
    utime.sleep_ms(1000)
    #=========================

    # 爪子張開
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=100, increments=50)
    utime.sleep_ms(1000)
    #==========================

    #手臂回到輸送带2============
    print("手臂歸位")   
    # 小臂下壓
    print("小臂下壓")
    set_servo_angle(pwm4, 150, delay=20, increments=50)  
    
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 220, delay=70, increments=50)
    utime.sleep_ms(1000)     

    # 小臂下壓
    print("小臂下壓")
    set_servo_angle(pwm4, 170, delay=20, increments=50)

    #=========================
    # 爪子關上
    print("爪子關上") 
    set_servo_angle(pwm3, 180, delay=50, increments=50)
    utime.sleep_ms(1000)
    #=========================   

def arm3_action():
    """ARM3 動作序列：從圓盤到綠色包裝"""
    print("執行 ARM3 動作")
    #開始動作=================
    print("開始動作")     
    # 底座 20度
    print("底座移動到圓盤")
    set_servo_angle(pwm, 20, delay=100, increments=50)
    #===============================================

    #到辨識圓盤夾取====================
    print("到辨識圓盤夾取")
    # 爪子張開===================
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=100, increments=50)
    #==========================
    
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 160, delay=20, increments=50)
    
    #大臂下壓
    print("大臂下壓")
    set_servo_angle(pwm2, 130, delay=50, increments=50)
    set_servo_angle(pwm2, 120, delay=40, increments=50)
    set_servo_angle(pwm2, 110, delay=40, increments=50)
    
     #爪子抓取===================
    print("爪子抓取")
    # 爪子閉合
    print("爪子閉合") 
    set_servo_angle(pwm3, 120, delay=100, increments=50)
       
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 190, delay=70, increments=50)
    utime.sleep_ms(500)
    
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 140, delay=70, increments=50)
    
    #轉到綠色包裝=========================
    print("轉到綠色包裝")
    # 底座 -8度
    print("底座轉到 -8 度") 
    set_servo_angle(pwm, -8, delay=100, increments=50)
    utime.sleep_ms(2000)
    
    #大臂下壓
    print("大臂下壓")
    set_servo_angle(pwm2, 180, delay=70, increments=50)

    # 小臂下壓(角度變大)
    print("小臂下壓")
    set_servo_angle(pwm4, 160, delay=70, increments=50)      
    utime.sleep_ms(1000)

    #=========================
    # 爪子張開
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=100, increments=50)
    utime.sleep_ms(1000)
    #==========================

    #手臂回到輸送带2============
    print("手臂回到輸送带2")
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 130, delay=30, increments=50)
        
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 220, delay=70, increments=50)
    
    # 小臂下壓
    print("小臂下壓")
    set_servo_angle(pwm4, 170, delay=30, increments=50)
    utime.sleep_ms(1000)

    #=========================
    # 爪子關上
    print("爪子關上") 
    set_servo_angle(pwm3, 180, delay=50, increments=50)
    utime.sleep_ms(1000)
    #=========================

def arm4_action():
    """ARM4 動作序列：從圓盤到輸送帶"""
    print("執行 ARM4 動作")   
    #開始動作=================
    print("開始動作")     
    # 底座 20度
    print("底座移動到圓盤")
    set_servo_angle(pwm, 20, delay=100, increments=50)
    #===============================================

    #到辨識圓盤夾取====================
    print("到辨識圓盤夾取")
    # 爪子張開===================
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=100, increments=50)
    #==========================
    
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 160, delay=20, increments=50)
    
    #大臂下壓
    print("大臂下壓")
    set_servo_angle(pwm2, 130, delay=50, increments=50)
    set_servo_angle(pwm2, 120, delay=40, increments=50)
    set_servo_angle(pwm2, 110, delay=40, increments=50)
    
     #爪子抓取===================
    print("爪子抓取")
    # 爪子閉合
    print("爪子閉合") 
    set_servo_angle(pwm3, 120, delay=100, increments=50)
       
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 190, delay=70, increments=50)
    utime.sleep_ms(500)

    # 底座移動到輸送帶=================================
    # 底座 125度
    print("底座轉到 125 度") 
    set_servo_angle(pwm, 125, delay=100, increments=50)
    
    # 小臂上抬、大臂下壓
    print("小臂上抬")
    set_servo_angle(pwm4, 150, delay=50, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 180, delay=50, increments=50)
    # 小臂下壓、大臂下壓
    print("小臂下壓")
    set_servo_angle(pwm4, 154, delay=20, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 176, delay=30, increments=50)
    print("小臂下壓")
    set_servo_angle(pwm4, 162, delay=20, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 170, delay=30, increments=50)
    print("小臂下壓")
    print("大臂下壓")
    set_servo_angle(pwm2, 156, delay=30, increments=50)
    set_servo_angle(pwm2, 150, delay=30, increments=50)
    print("大臂下壓")
    set_servo_angle(pwm2, 144, delay=30, increments=50)
    set_servo_angle(pwm2, 140, delay=30, increments=50)
    set_servo_angle(pwm2, 135, delay=30, increments=50)
    set_servo_angle(pwm2, 130, delay=30, increments=50)
    set_servo_angle(pwm2, 127, delay=30, increments=50)

    #=========================
    # 爪子張開
    print("爪子張開") 
    set_servo_angle(pwm3, 60, delay=100, increments=50)
    utime.sleep_ms(1000)
    #==========================
    
    # 大臂逐步回到開始位 
    print("大臂回到開始位")
    set_servo_angle(pwm2, 220, delay=100, increments=50)
    
    # 小臂上抬
    print("小臂上抬")
    set_servo_angle(pwm4, 170, delay=100, increments=50)
    utime.sleep_ms(1000)

    #=========================
    # 爪子關上
    print("爪子關上") 
    set_servo_angle(pwm3, 180, delay=100, increments=50)
    utime.sleep_ms(1000)
    #=========================

def measure_distance():
    """測量超音波感測器的距離"""
    trigger_pin.value(0)
    time.sleep(0.002)
    trigger_pin.value(1)
    time.sleep(0.00001)
    trigger_pin.value(0)
    
    try:
        duration = time_pulse_us(echo_pin, 1, 30000)
        if duration < 0:
            return None
        distance_cm = (duration / 1000000 * 34000) / 2
        return distance_cm if distance_cm >= 1 else None
    except Exception as e:
        print("測量錯誤:", e)
        return None

def safe_stop_motor():
    """安全地停止輸送帶馬達"""
    for _ in range(3):
        relay_pin.value(0)
        time.sleep(0.1)

def distance_thread():
    """距離測量線程"""
    global current_distance, stop_threads, conveyor_control
    
    while not stop_threads:
        if conveyor_control == 'g':
            try:
                distance = measure_distance()
                if distance is not None:
                    with lock:
                        current_distance = round(distance, 1)
                time.sleep(0.05)
            except Exception as e:
                print("距離測量錯誤:", e)
                time.sleep(1)
        else:
            time.sleep(0.1)

def conveyor_thread():
    """輸送帶控制線程"""
    global current_distance, stop_threads, conveyor_control
    
    while not stop_threads:
        if conveyor_control == 'g':
            try:
                with lock:
                    distance = current_distance
                if distance is not None and distance < 6:
                    relay_pin.value(0)
                    conveyor_control = 's'
                else:
                    relay_pin.value(1)
                time.sleep(0.1)
            except Exception as e:
                print("輸送帶控制錯誤:", e)
                safe_stop_motor()
                time.sleep(1)
        else:
            safe_stop_motor()
            time.sleep(0.1)

def set_servo_angle_Package(servo, angle, current_angle, delay=20, increments=50):
    # 計算開始和結束的duty比
    start_duty = int(MIN_DUTY_Package + (current_angle / 180) * (MAX_DUTY_Package - MIN_DUTY_Package))
    end_duty = int(MIN_DUTY_Package + (angle / 180) * (MAX_DUTY_Package - MIN_DUTY_Package))
    
    # 計算duty的step
    step = (end_duty - start_duty) / increments
    
    # 慢慢改變duty來完成緩慢轉動
    for i in range(increments):
        new_duty = int(start_duty + i * step)
        servo.duty(new_duty)
        utime.sleep_ms(delay)
    
    # 確保最終到達精確的目標位置
    servo.duty(end_duty)
    
    return angle  # 返回新的當前角度

def read_with_timeout(pin, timeout_ms):
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < timeout_ms:
        try:
            value = pin.read()
            return value
        except:
            pass
    return None  # 超時後返回 None

def get_initial_averages_green():
    global green_avg1, green_avg2, green_avg3
    """取得綠色包裝系統初始的10組平均值"""
    print("開始取得綠色系統初始校正值...")
    print("收集10組數據中...")
    
    values1 = []
    values2 = []
    values3 = []
    
    for i in range(10):
        sensor_value1 = read_with_timeout(green_ldr_pin1, 5000)
        sensor_value2 = read_with_timeout(green_ldr_pin2, 5000)
        sensor_value3 = read_with_timeout(green_ldr_pin3, 5000)
        
        values1.append(sensor_value1)
        values2.append(sensor_value2)
        values3.append(sensor_value3)
        
        print(f"第 {i+1} 組:")
        print(f"綠色系統光線感測值1: {sensor_value1}")
        print(f"綠色系統光線感測值2: {sensor_value2}")
        print(f"綠色系統光線感測值3: {sensor_value3}")
        time.sleep_ms(500)
    
    avg1 = sum(values1) / len(values1)
    avg2 = sum(values2) / len(values2)
    avg3 = sum(values3) / len(values3)
    
    print("\n綠色系統初始校正完成！")
    print(f"平均光線感測值1: {avg1:.1f}")
    print(f"平均光線感測值2: {avg2:.1f}")
    print(f"平均光線感測值3: {avg3:.1f}")
    print("------------------------")
    
    return avg1, avg2, avg3

def get_initial_averages_red():
    global red_avg1, red_avg2, red_avg3
    """取得紅色包裝系統初始的10組平均值"""
    print("開始取得紅色系統初始校正值...")
    print("收集10組數據中...")
    
    values1 = []
    values2 = []
    values3 = []
    
    for i in range(10):
        sensor_value1 = read_with_timeout(red_ldr_pin1, 5000)
        sensor_value2 = read_with_timeout(red_ldr_pin2, 5000)
        sensor_value3 = read_with_timeout(red_ldr_pin3, 5000)
        
        values1.append(sensor_value1)
        values2.append(sensor_value2)
        values3.append(sensor_value3)
        
        print(f"第 {i+1} 組:")
        print(f"紅色系統光線感測值1: {sensor_value1}")
        print(f"紅色系統光線感測值2: {sensor_value2}")
        print(f"紅色系統光線感測值3: {sensor_value3}")
        time.sleep_ms(500)
    
    avg1 = sum(values1) / len(values1)
    avg2 = sum(values2) / len(values2)
    avg3 = sum(values3) / len(values3)
    
    print("\n紅色系統初始校正完成！")
    print(f"平均光線感測值1: {avg1:.1f}")
    print(f"平均光線感測值2: {avg2:.1f}")
    print(f"平均光線感測值3: {avg3:.1f}")
    print("------------------------")
    
    return avg1, avg2, avg3

def update_green_servo(sensor_value1, sensor_value2, sensor_value3, current_angle):
    """更新綠色伺服馬達的角度"""
    new_angle = current_angle  # 預設保持當前位置

    # 條件判斷
    if sensor_value1 <= 1000:
        print("綠色系統條件一觸發")
        print(f"sensor_value1: {sensor_value1} <= 1000")
        new_angle = 80
    if sensor_value2 <= 1000:
        print("綠色系統條件二觸發")
        print(f"sensor_value2: {sensor_value2} <= 1000")
        new_angle = 165
    if sensor_value3 <= 1000:
        print("綠色系統條件三觸發")
        print(f"sensor_value3: {sensor_value3} <= 1000")
        new_angle = 270
        print("綠色系統即將滿倉")
    if sensor_value1 >= 3000 and sensor_value2 >= 3000 and sensor_value3 >= 2500:
        print("綠色系統條件四觸發")
        print(f"數值確認：")
        print(f"sensor_value1: {sensor_value1} >= 3000")
        print(f"sensor_value2: {sensor_value2} >= 3000")
        print(f"sensor_value3: {sensor_value3} >= 2500")
        print("綠色系統回正")
        new_angle = 0
    else:
        print("綠色系統條件五觸發 - 不做任何動作")

    # 只在角度需要改變時才移動伺服馬達
    if new_angle != current_angle:
        print(f"移動綠色伺服馬達: 從 {current_angle}° 到 {new_angle}°")
        current_angle = set_servo_angle_Package(pwm6_Package, new_angle, current_angle)
        print(f"移動完成，綠色系統當前角度: {current_angle}°")
    
    return current_angle

def update_red_servo(sensor_value1, sensor_value2, sensor_value3, current_angle):
    """更新紅色伺服馬達的角度"""
    new_angle = current_angle  # 預設保持當前位置

    # 條件判斷
    if sensor_value1 <= 1000:
        print("紅色系統條件一觸發")
        print(f"sensor_value1: {sensor_value1} <= 1000")
        new_angle = 80
    if sensor_value2 <= 1000:
        print("紅色系統條件二觸發")
        print(f"sensor_value2: {sensor_value2} <= 1000")
        new_angle = 165
    if sensor_value3 <= 1000:
        print("紅色系統條件三觸發")
        print(f"sensor_value3: {sensor_value3} <= 1000")
        new_angle = 270
        print("紅色系統即將滿倉")
    if sensor_value1 >= 3000 and sensor_value2 >= 3000 and sensor_value3 >= 2500:
        print("紅色系統條件四觸發")
        print(f"數值確認：")
        print(f"sensor_value1: {sensor_value1} >= 3000")
        print(f"sensor_value2: {sensor_value2} >= 3000")
        print(f"sensor_value3: {sensor_value3} >= 2500")
        print("紅色系統回正")
        new_angle = 0
    else:
        print("紅色系統條件五觸發 - 不做任何動作")

    # 只在角度需要改變時才移動伺服馬達
    if new_angle != current_angle:
        print(f"移動紅色伺服馬達: 從 {current_angle}° 到 {new_angle}°")
        current_angle = set_servo_angle_Package(pwm5_Package, new_angle, current_angle)
        print(f"移動完成，紅色系統當前角度: {current_angle}°")
    
    return current_angle

def Package_auto():
    global green_avg1, green_avg2, green_avg3, red_avg1, red_avg2, red_avg3
    global package_thread_running  
    print("開始整合包裝系統監測...")
    # 使用全局變量而不是重新計算
    print(f"使用綠色系統初始平均值: {green_avg1:.1f}, {green_avg2:.1f}, {green_avg3:.1f}")
    print(f"使用紅色系統初始平均值: {red_avg1:.1f}, {red_avg2:.1f}, {red_avg3:.1f}")
    # 初始化伺服馬達位置
    initial_duty = int(MIN_DUTY_Package + (0 / 180) * (MAX_DUTY_Package - MIN_DUTY_Package))
    pwm5_Package.duty(initial_duty)  # 紅色伺服馬達設置初始位置
    pwm6_Package.duty(initial_duty)  # 綠色伺服馬達設置初始位置
    time.sleep(1)  # 給伺服馬達足夠時間到達初始位置
    
    # 追蹤當前角度
    red_current_angle = 0
    green_current_angle = 0
    
    print("\n開始持續監測...")
    try:
        while package_thread_running:
            print("\n-----綠色包裝系統-----")
            # 讀取綠色系統當前值
            green_sensor_value1 = green_ldr_pin1.read()
            green_sensor_value2 = green_ldr_pin2.read()
            green_sensor_value3 = green_ldr_pin3.read()
            
            # 輸出綠色系統結果
            print("\n綠色系統當前讀數:")
            print(f"光線感測值1: {green_sensor_value1} (初始平均: {green_avg1:.1f})")
            print(f"光線感測值2: {green_sensor_value2} (初始平均: {green_avg2:.1f})")
            print(f"光線感測值3: {green_sensor_value3} (初始平均: {green_avg3:.1f})")
            
            # 打印綠色系統詳細數值
            print("\n詳細的綠色系統感測器數值：")
            print(f"當前角度: {green_current_angle}")
            print(f"sensor_value1: {green_sensor_value1}")
            print(f"sensor_value2: {green_sensor_value2}")
            print(f"sensor_value3: {green_sensor_value3}")
            
            # 更新綠色伺服馬達
            green_current_angle = update_green_servo(
                green_sensor_value1, 
                green_sensor_value2, 
                green_sensor_value3, 
                green_current_angle
            )
            
            print("\n-----紅色包裝系統-----")
            # 讀取紅色系統當前值
            red_sensor_value1 = red_ldr_pin1.read()
            red_sensor_value2 = red_ldr_pin2.read()
            red_sensor_value3 = red_ldr_pin3.read()
            
            # 輸出紅色系統結果
            print("\n紅色系統當前讀數:")
            print(f"光線感測值1: {red_sensor_value1} (初始平均: {red_avg1:.1f})")
            print(f"光線感測值2: {red_sensor_value2} (初始平均: {red_avg2:.1f})")
            print(f"光線感測值3: {red_sensor_value3} (初始平均: {red_avg3:.1f})")
            
            # 打印紅色系統詳細數值
            print("\n詳細的紅色系統感測器數值：")
            print(f"當前角度: {red_current_angle}")
            print(f"sensor_value1: {red_sensor_value1}")
            print(f"sensor_value2: {red_sensor_value2}")
            print(f"sensor_value3: {red_sensor_value3}")
            
            # 更新紅色伺服馬達
            red_current_angle = update_red_servo(
                red_sensor_value1, 
                red_sensor_value2, 
                red_sensor_value3, 
                red_current_angle
            )
            
            # 延遲1秒
            time.sleep(1)
            
    except Exception as e:
        print(f"\n包裝系統監測錯誤: {e}")
    finally:
        print("\n包裝系統監測結束")

def package_thread_entry():
    """包裝系統監測執行緒入口點"""
    try:
        Package_auto()
    except Exception as e:
        print(f"包裝系統執行緒錯誤: {e}")

def start_server():
    """啟動 Socket 伺服器並處理客戶端命令"""
    global conveyor_control, stop_threads, s
    
    # 啟動輸送帶相關線程
    _thread.start_new_thread(distance_thread, ())
    _thread.start_new_thread(conveyor_thread, ())
    
    while True:
        cl, addr = s.accept()
        print('客戶端連接來自:', addr)
        
        try:
            while True:
                data = cl.recv(1024).decode('utf-8').strip()
                if not data:
                    break
                
                response = "命令已執行"
                
                if data == '1':
                    # ARM1 動作序列
                    arm1_action()
                elif data == '2':
                    # ARM2 動作序列
                    arm2_action()
                elif data == '3':
                    # ARM3 動作序列
                    arm3_action()
                elif data == '4':
                    # ARM4 動作序列
                    arm4_action()
                elif data == 'c':
                    # 控制輸送帶
                    if conveyor_control == 's':
                        conveyor_control = 'g'
                        response = "輸送帶啟動"
                    else:
                        conveyor_control = 's'
                        response = "輸送帶停止"
                elif data == 'k':
                    # 控制分類圓盤
                    set_continuous_speed(15)
                    response = "已執行"
                elif data == 'x':
                    # 機械手臂校正
                    arm_calibration()
                    response = "機械手臂校正完成"
                elif data == 'p':
                    # 包裝系統測試
                    Package_auto()
                    response = "包裝系統測試完成"
                elif data == 'y':
                    # 這裡只返回一個確認回應，實際影像處理在 PC 端進行
                    response = "影像辨識命令已接收"
                elif data == 's':
                    # 停止所有動作
                    conveyor_control = 's'
                    safe_stop_motor()
                    set_continuous_speed(0)
                    pwm7.duty(0)
                    response = "馬達已暫停"
                elif data == 'f':
                    # 停止所有動作
                    conveyor_control = 's'
                    safe_stop_motor()
                    set_continuous_speed(0)
                    pwm7.duty(0)
                    # 底座回正
                    print("底座回正") 
                    set_servo_angle(pwm, 90, delay=100, increments=50)#底座
                    utime.sleep_ms(1000)
                    # 爪子回正
                    print("爪子回正") 
                    set_servo_angle(pwm3, 200, delay=100, increments=50)
                    utime.sleep_ms(500)
                    # 大臂逐步回到開始位 
                    print("大臂回到開始位")
                    set_servo_angle(pwm2, 220, delay=30, increments=50)
                    utime.sleep_ms(500)
                    # 小臂回開始位 
                    print("小臂回開始位")
                    set_servo_angle(pwm4, 170, delay=30, increments=50)
                    utime.sleep_ms(500)
                    response = "所有系統已停止"
                
                
                cl.send(f"{response}\n".encode('utf-8'))
                
        except Exception as e:
            print('錯誤:', e)
        finally:
            cl.close()

#===================#

def main():
    global green_avg1, green_avg2, green_avg3, red_avg1, red_avg2, red_avg3
    global package_thread_running  
    """主程式"""
    try:
        connect_wifi()
        utime.sleep_ms(1000)
        # 在連接 WiFi 後獲取初始平均值
        print("正在獲取初始平均值...")
        green_avg1, green_avg2, green_avg3 = get_initial_averages_green()
        red_avg1, red_avg2, red_avg3 = get_initial_averages_red()
        utime.sleep_ms(1000)
        
        # 啟動包裝系統監測執行緒
        package_thread_running = True
        _thread.start_new_thread(package_thread_entry, ())
        print("包裝系統監測已在背景啟動")
        
        # 啟動 Socket 伺服器
        start_server()
                           
    except KeyboardInterrupt:
        print("\n程式終止中...")
    except Exception as e:
        print("主程式錯誤:", e)
    finally:
        # 停止所有執行緒
        package_thread_running = False
        stop_threads = True
        # 等待一小段時間讓執行緒有機會終止
        time.sleep(1)
        safe_stop_motor()
        set_continuous_speed(0)
        pwm.duty(0) #設定輸出為零
        pwm2.duty(0) #設定輸出為零
        pwm3.duty(0) #設定輸出為零
        pwm4.duty(0) #設定輸出為零
        pwm5_Package.duty(0) #設定輸出為零
        pwm6_Package.duty(0) #設定輸出為零
        pwm7.duty(0) #設定輸出為零

if __name__ == '__main__':
    main()