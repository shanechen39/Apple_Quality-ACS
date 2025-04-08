#PC環境除錯端
import socket
import sys
import cv2
import torch
import time
from collections import Counter
from ultralytics import YOLO
import threading

# 連接設定
ESP32_IP = ""  # 請替換成你的 ESP32 IP 位址
PORT = 12345

#全域變數宣告
result = None

def connect_to_esp32():
    """建立與 ESP32 的連接"""
    s = socket.socket()
    try:
        s.connect((ESP32_IP, PORT))
        return s
    except Exception as e:
        print(f"連接失敗: {e}")
        sys.exit(1)

def load_model():
    # 載入預訓練的YOLO模型
    model = YOLO("best.pt")
    return model

def capture_and_predict(model, cap):
    # 捕獲一幀影像
    ret, frame = cap.read()
    if not ret:
        return None, None
    
    # 使用模型進行預測
    results = model(frame)
    
    # 檢查是否有檢測結果
    if len(results[0].boxes) > 0:
        # 獲取所有檢測結果中置信度最高的那個
        confidences = results[0].boxes.conf
        classes = results[0].boxes.cls
        boxes = results[0].boxes.xyxy  # 獲取邊界框座標
        
        # 找到最高置信度的索引
        max_conf_idx = confidences.argmax()
        class_id = int(classes[max_conf_idx])
        
        # 獲取最高置信度檢測的邊界框座標
        box = boxes[max_conf_idx]
        x1, y1, x2, y2 = map(int, box.tolist())
        
        # 計算邊界框的寬度和高度
        width = x2 - x1
        height = y2 - y1
        
        # 將數字類別ID轉換為對應的字母類別
        class_mapping = {0: 'A', 1: 'B', 2: 'C'}
        predicted_class = class_mapping.get(class_id, None)
        
        # 創建包含邊界框信息的字典
        detection_info = {
            'class': predicted_class,
            'confidence': float(confidences[max_conf_idx]),
            'bbox': {
                'x1': x1,
                'y1': y1,
                'x2': x2,
                'y2': y2,
                'width': width,
                'height': height
            }
        }
        
        return predicted_class, detection_info
    return None, None

def YOLOCD03():
    global result
    # 初始化攝像頭
    print("攝像頭啟動中...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("無法開啟攝像頭")
        return
    
    # 載入模型
    model = load_model()
    
    # 倒數5秒
    #print("準備開始偵測")
    #for i in range(5, 0, -1):
        #print(f"倒數 {i} 秒...")
        #time.sleep(1)
    
    print("開始偵測!")
    
    # 儲存預測結果
    predictions = []
    detection_records = []
    target_predictions = 6
    interval = 0.6
    no_prediction_count = 0  # 新增：計算連續無預測的次數

    while len(predictions) < target_predictions:
        prediction, detection_info = capture_and_predict(model, cap)
        if prediction and detection_info:
            predictions.append(prediction)
            detection_records.append(detection_info)
            no_prediction_count = 0  # 重置計數器
        else:
            no_prediction_count += 1  # 增加計數器
            if no_prediction_count >= 10:
                print("沒有偵測到蘋果，請檢查系統")
                cap.release()
                return  # 終止函數執行
        time.sleep(interval)
    
    # 釋放攝像頭
    cap.release()
    
    # 統計結果
    if not predictions:
        print("沒有有效的預測結果")
        return
    
    # 計算每個類別的出現次數
    counter = Counter(predictions)
    result = None
    
    # 根據規則判斷最終結果
    if counter.get('C', 0) >= 1:
        result = 'C'
    if counter.get('A', 0) >= 3:
        result = 'A'
    if counter.get('B', 0) >= 3:
        result = 'B'
    if counter.get('C', 0) >= 3:
        result = 'C'
    
    # 輸出結果
    print("\n詳細檢測記錄:")
    for i, record in enumerate(detection_records, 1):
        bbox = record['bbox']
        print(f"檢測 {i}:")
        print(f"  類別: {record['class']}")
        print(f"  '相似度: {record['confidence']:.3f}")
        print(f"  邊界框座標: 左上({bbox['x1']}, {bbox['y1']}) 右下({bbox['x2']}, {bbox['y2']})")
        print(f"  邊界框大小: {bbox['width']}x{bbox['height']} 像素")
        print()
    
    #print(f"檢測類別序列: {predictions}")
    print(f"最終結果: {result if result else '無法判定'}")

    return result

def execute_command(socket, command):
    """執行單個命令並處理回應"""
    print(f"\n執行命令: {command}")
    socket.send(command.encode('utf-8'))
    response = socket.recv(1024).decode('utf-8')
    print("ESP32 回應:", response.strip())
    time.sleep(1)  # 在每個命令之間添加短暫延遲

def main():
    """主程式"""
    print("正在連接到 ESP32...")
    s = connect_to_esp32()
    print("連接成功！")
    print("系統正在啟動...")
    time.sleep(2)
    print("系統啟動成功，如有問題請檢查電源供應!") 
    time.sleep(1)   
    try:
        # 依序執行命令
        print("執行機械手臂校正")
        execute_command(s, 'x')
        time.sleep(1)
        print("輸送帶開啟")
        execute_command(s, 'c')
        time.sleep(10)
        print("機械手臂夾取待測蘋果到辨識平台")
        execute_command(s, '1')
        time.sleep(1) 
        
        print("開始YOLOv8影像辨識")
        
        # 建立一個執行緒來執行 YOLOCD03()
        def run_yolo():
            global result
            result = YOLOCD03()
            
        yolo_thread = threading.Thread(target=run_yolo)
        yolo_thread.daemon = True  # 設定為守護執行緒，主程式結束時會自動結束
        yolo_thread.start()
        
        # 主執行緒等待 14 秒並啟動圓盤
        time.sleep(14)
        #print("辨識平台圓盤馬達啟動")
        execute_command(s, 'k')
        
        # 等待 3 秒後停止圓盤
        time.sleep(3)
        
        # 等待 YOLO 執行緒完成
        yolo_thread.join()
        
        print("YOLOv8影像辨識完畢")        
        # 停止圓盤轉動
        execute_command(s, 's')
        time.sleep(1)
        
        # 根據辨識結果執行相應動作
        if result == 'C':
            print("機械手臂夾取C類蘋果到輸送帶")
            execute_command(s, '4')
            time.sleep(1)
            # 確保輸送帶是停止的
            execute_command(s, 'c')  # 如果輸送帶是啟動的，這會停止它
            time.sleep(0.5)          # 短暫等待
            print("輸送帶運送C類蘋果到藍色包裝區")
            execute_command(s, 'c')
            print("正在運送C類蘋果到藍色包裝區...")
            time.sleep(8)
            execute_command(s, 'c')  #輸送帶停止
            time.sleep(1)
        elif result == 'A':  # 使用 elif 避免多次判斷
            print("機械手臂夾取A類蘋果到紅色包裝區")
            print("正在運送A類蘋果到紅色包裝區")
            execute_command(s, '2')
            time.sleep(1)     
        elif result == 'B':  # 使用 elif 避免多次判斷
            print("機械手臂夾取B類蘋果到綠色包裝區")
            print("正在運送B類蘋果到綠色包裝區")
            execute_command(s, '3')
            time.sleep(1)
        else:
            print(f"辨識結果為 {result}，無法確定正確的動作")
            
        print("\n所有命令執行完畢")
    
    except Exception as e:
        print(f"錯誤: {e}")
        execute_command(s, 's')
    finally:
        print("系統關閉中...")
        execute_command(s, 's')
        execute_command(s, 'f')
        print("系統已關閉，如無需使用請關閉電源!")
        s.close()

if __name__ == "__main__":
    main()