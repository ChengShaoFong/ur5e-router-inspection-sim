# 雙手臂 Router 清潔任務

開啟 `worlds/Neura_Dual_Arm_Router_Service.wbt`，在 Webots R2025a 執行模擬。這是獨立場景，不會改動原本的 RJ45 線材任務。

場景有一台雙網路孔 Router、兩台 UR5e、一個中空轉接插座，以及長條形清潔頭。預設執行 **port A**；可在 `task.ini` 指定多個點位順序。插座的公頭朝 +X，後方四片外殼圍出可供清潔頭進入的通道。

## 最簡啟動與新增點位

用 Webots 開啟 `worlds/Neura_Dual_Arm_Router_Service.wbt` 並按執行即可；不需要傳入額外參數。若要依序執行 A、B，將 `task.ini` 的 `[task]` 區段改為 `run_order = ["port_a", "port_b"]`，再重設模擬。同區段的 `port_a`、`port_b` 是插孔目標中心的世界座標 `[x, y, z]`，單位公尺。第一站從地面取插座；中間站拆下後，手臂一夾著插座直接移到下一站；最後一站結束才放回地面。

新增實體插孔時，先在世界檔建立插孔外形及同名鎖扣 `port_x_latch`，再於 `task.ini` 的 `[task]` 加入 `port_x = [x, y, z]` 並將 `"port_x"` 放進 `run_order`。`[calibration]` 保留工具和夾爪的現場校正；各點位共用相同校正偏移。點位順序不可重複。新增相距較遠或朝向不同的插孔時，仍需檢查手臂可達性、相機視野及避障路徑。

流程以模擬時間排定：

1. 手臂 1 用 Robotiq 夾爪取起插座，插入 port A，鬆開後回到待命姿態。
2. 手臂 2 用安裝在清潔頭座上的相機定位插座開口，修正清潔頭路徑，再伸入插座、沿插入軸旋轉 180°，拔出後回到待命姿態。
3. 手臂 1 再次夾取插座，等待插孔解除鎖定後拔出；若設定了下一點位則直接搬過去，否則放回地面。各動作秒數由 `plan.py` 根據點位順序產生。

兩支手臂的關節由逆向運動學路徑控制。插座有質量與碰撞外形，會受重力與接觸力影響。手臂 1 的夾爪沒有抓取用 `Connector`：指墊靠碰撞、關節扭力及摩擦夾住插座，控制器不會直接改寫插座位置。清潔頭也有質量與碰撞外形。Router 的 port A/B 使用 `Connector` 模擬 RJ45 卡扣；這是理想化的插孔固定，尚未模擬彈片變形或真實清潔效果。

現場可調整 `task.ini` 的 `[calibration]`：`pickup_tip`（地面抓取）、`installed_tip`（插孔抓取）、`insertion_tip`（插入位置）、`place_tip`（放回地面）與 `closed_angle`（夾爪閉合角度）。座標單位為公尺，角度單位為弧度。先在 Webots 場景樹確認插座及插孔的位置，再小量調整 x、y、z；尤其要讓兩片指墊落在插座兩側。修改 INI 後重設模擬。閉爪與抬升之間的停頓由 `plan.py` 的 `hold while fingers close` 設定；手指關節停穩且插座未大幅移位後，才開始抬升。手臂 1 在插入前會量測插座相對夾爪的位置並校正插入目標。若夾取失敗，手臂 1 停在原處並於 Console 顯示 `FAILED`；若插座未裝上，手臂 2 不會開始清潔。

可先執行 `python tools/verify_dual_arm_plan.py` 檢查目標可達性、動作順序與路徑連續性。完整的夾取、接觸與插孔卡扣行為需在 Webots 內執行確認。

## Port A 視覺對位

`port_a_camera` 安裝在手臂 1 的夾爪座上。手臂在 18 秒移到 Port A 前方並停住，18.5–20 秒由相機拍照定位；Webots 的 `port_a_vision_display` 顯示影像，綠框是 AOI，青色框是偵測到的插孔邊框。20 秒確認定位成功後才開始插入。若畫面未出現，可在 Webots 的 Overlays / Display Devices 選取該 Display。20 秒會儲存原始相機影像至 `.dual-arm-runtime/port_a_capture.png`；設定 `DUAL_ARM_TRACE_DIR` 時會存入指定目錄。

手臂 1 控制器從相機像素偵測插孔的亮色中空外框，排除插孔上方的亮色橫條，並用相機即時姿態把像素射線投影到外框前平面。連續五張影像的位置變動小於 2.5 mm 才採用 y/z 中位數。插入時仍使用 `task.ini` 的 x 深度，並保留夾持偏移校正；沒有可靠的視覺定位就停止插入。此版是針對目前 Webots 場景的影像規則辨識，**尚未使用訓練過的 AI 模型**。若要換成模型推論，需要插孔影像、標註與模型檔，並在 Webots 裡驗證遮擋及誤判。

## 手臂二插座視覺追蹤

`adapter_socket_camera` 固定在手臂二清潔頭座側上方，鏡頭朝向插座後方開口；`adapter_socket_vision_display` 顯示影像、綠色 AOI 和青色辨識框。手臂二在 34.5–41 秒接近插座期間持續拍照。辨識器找橘色中空外框，用已知的 50 mm 外框寬度和影像像素寬度估計距離，再用相機姿態計算插座 x/y/z，因此插座中心不必固定在原本的標稱位置。

目前使用影像顏色與外框幾何辨識，尚未使用訓練過的 AI 模型。若未來改用模型，可替換 `adapter_vision.py` 並維持相同的 `locate_adapter` 輸入與輸出。

連續三張影像定位穩定後，若插座位置變動超過 3 mm，控制器會重新計算接近、插入、旋轉及退出路徑；更新間隔至少 0.4 秒。每站插入前必須有近期相機定位，否則手臂二停止。插入後清潔頭可能遮住開口，因此視覺追蹤只在接近與插入前更新路徑。原始影像以 `<點位>_adapter_socket_capture.png` 儲存至 `.dual-arm-runtime`；設定 `DUAL_ARM_TRACE_DIR` 時則存入指定目錄。

## 程式架構與修改位置

| 檔案 | 負責內容 | 常見修改 |
| --- | --- | --- |
| `task.ini` | 點位、校正、控制門檻及兩支相機參數 | 新增點位、調整 AOI 與夾爪校正 |
| `settings.py` | 讀取 INI 並提供各模組所需區段 | 增加新的設定區段 |
| `plan.py` | 兩支手臂的動作時間、目標位置與流程事件 | 調整移動時機、姿態與路徑 |
| `kinematics.py` | UR5e 正向、逆向運動學 | 更換機器人模型或工具尺寸 |
| `dual_arm_router_task.py` | 初始化設備，執行主循環、夾持檢查與插入校正 | 調整任務流程與失敗條件 |
| `port_vision.py` | 單張影像找插孔，輸出世界 y/z、外框、信心值 | 替換為 OpenCV 或 AI 模型 |
| `vision_tracker.py` | 相機取樣、畫框、保存影像及多張影像確認 | 調整取樣與顯示方式 |
| `adapter_vision.py` | 從手臂二影像估計插座三維位置 | 替換插座辨識方法 |
| `adapter_vision_tracker.py` | 手臂二相機取樣、畫框、連續影像確認 | 調整追蹤方式 |
| `vision_geometry.py` | 兩支相機共用的像素射線與投影計算 | 更換相機投影模型 |

每個模擬步驟的順序是：**讀取相機與手指感測器 → 處理時間表事件 → 檢查物件位置 → 下達關節目標 → 回報結果**。`plan.py` 的 `CueEvent` 用於流程判斷，`label` 只用於顯示；修改顯示文字不會影響事件觸發。

調整手臂一 AOI 時修改 `task.ini` 的 `[port_vision]`，手臂二則修改 `[adapter_vision]`；格式都是 `aoi = [左, 上, 右, 下]`。影像解析度與視角必須和世界檔對應的 Camera 一致；若移動手臂二相機，還要同步修改 `camera_tool_translation` 和 `camera_tool_y_rotation`，讓離線視野檢查反映場景。手臂一的取樣起點由 `plan.py` 的停靠事件加上 `settle_seconds` 決定，終點為拍照事件。若要換辨識方法，讓新模組提供與 `port_vision.locate_port(image, camera_position, camera_orientation, port_plane_x)` 或 `adapter_vision.locate_adapter(image, camera_position, camera_orientation)` 相同的介面，再改對應 tracker 的匯入即可；夾爪與任務時間表不需一起改。

離線檢查可執行 `python -B -m unittest discover -s tools -p 'test_*.py'` 與 `python -B tools/verify_dual_arm_plan.py`。完整接觸、卡扣和畫面仍需在 Webots 中確認。
