# 雙手臂 Router 清潔任務

開啟 `worlds/Neura_Dual_Arm_Router_Service.wbt`，在 Webots R2025a 執行模擬。這是獨立場景，不會改動原本的 RJ45 線材任務。

場景有一台雙網路孔 Router、兩台 UR5e、一個中空轉接插座，以及長條形清潔頭。任務使用上方的 **port A**；下方的 **port B** 留作擴充。插座的公頭朝 +X，後方四片外殼圍出可供清潔頭進入的通道。

流程以模擬時間排定：

1. 手臂 1 用 Robotiq 夾爪取起插座，插入 port A，鬆開後回到待命姿態。
2. 手臂 2 將長條清潔頭伸入插座，沿插入軸旋轉 180°，拔出後回到待命姿態。
3. 手臂 1 再次夾取插座，等待插孔解除鎖定，從 port A 拔出，放回地面，最後回到待命姿態。各動作秒數以 `plan.py` 為準。

兩支手臂的關節由逆向運動學路徑控制。插座有質量與碰撞外形，會受重力與接觸力影響。手臂 1 的夾爪沒有抓取用 `Connector`：指墊靠碰撞、關節扭力及摩擦夾住插座，控制器不會直接改寫插座位置。清潔頭也有質量與碰撞外形。Router 的 port A 仍使用 `Connector` 模擬 RJ45 卡扣；這是理想化的插孔固定，尚未模擬彈片變形或真實清潔效果。

現場可調整 `calibration.json` 的 `pickup_tip`（地面抓取）、`installed_tip`（插孔抓取）、`insertion_tip`（插入位置）、`place_tip`（放回地面）與 `closed_angle`（夾爪閉合角度）。座標單位為公尺，角度單位為弧度。先在 Webots 場景樹確認插座及插孔的位置，再小量調整 x、y、z；尤其要讓兩片指墊落在插座兩側。修改 JSON 後重設模擬。閉爪與抬升之間的停頓由 `plan.py` 的 `hold while fingers close` 設定；手指關節停穩且插座未大幅移位後，才開始抬升。手臂 1 在插入前會量測插座相對夾爪的位置並校正插入目標。若夾取失敗，手臂 1 停在原處並於 Console 顯示 `FAILED`；若插座未裝上，手臂 2 不會開始清潔。

可先執行 `python tools/verify_dual_arm_plan.py` 檢查目標可達性、動作順序與路徑連續性。完整的夾取、接觸與插孔卡扣行為需在 Webots 內執行確認。

## Port A 視覺對位

`port_a_camera` 安裝在手臂 1 的夾爪座上。手臂在 18 秒移到 Port A 前方並停住，18.5–20 秒由相機拍照定位；Webots 的 `port_a_vision_display` 顯示影像，綠框是 AOI，青色框是偵測到的插孔邊框。20 秒確認定位成功後才開始插入。若畫面未出現，可在 Webots 的 Overlays / Display Devices 選取該 Display。20 秒會儲存原始相機影像至 `.dual-arm-runtime/port_a_capture.png`；設定 `DUAL_ARM_TRACE_DIR` 時會存入指定目錄。

手臂 1 控制器從相機像素偵測插孔的亮色中空外框，排除插孔上方的亮色橫條，並用相機即時姿態把像素射線投影到外框前平面。連續五張影像的位置變動小於 2.5 mm 才採用 y/z 中位數。插入時仍使用 `calibration.json` 的 x 深度，並保留夾持偏移校正；沒有可靠的視覺定位就停止插入。此版是針對目前 Webots 場景的影像規則辨識，**尚未使用訓練過的 AI 模型**。若要換成模型推論，需要插孔影像、標註與模型檔，並在 Webots 裡驗證遮擋及誤判。
