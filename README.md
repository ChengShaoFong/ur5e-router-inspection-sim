# 模擬範例

https://github.com/user-attachments/assets/23b2402d-593f-4822-a061-e1f9c1d7581a

# 雙手臂 Router 任務

用 Webots 開啟 `worlds/Neura_Dual_Arm_Router_Service.wbt`，按執行即可。任務會由手臂一取起插座、裝入指定插孔，手臂二定位並清潔插座，最後由手臂一拆下。多個點位會依順序執行；中間不會把插座放回地面。

## 力感測示波器

雙臂世界另有兩個力感測示波器，執行後顯示在 Webots 3D 視窗左側。紅／綠／藍分別為工具局部座標的 Fx／Fy／Fz，黃色為合力 `|F|`，單位 N。橫軸是模擬時間（秒），保留最近 10 秒；縱軸自動縮放，並顯示即時值及視窗內合力峰值。每 32 ms 取樣，約每 0.1 秒重畫，保留未平滑的瞬間尖峰。

量測位置為手臂一的夾爪座與手臂二的清潔頭，讀值是原始工具負載，包含重力、運動慣性和接觸反作用，未做重力補償；不是單片夾爪的夾持力。世界檔使用 `force-3d` TouchSensor 與空的 `lookupTable`，直接輸出 N（[Webots 官方說明](https://cyberbotics.com/doc/reference/touchsensor)）。

若波形被關閉，選取對應手臂，在 `Overlays → Display Devices` 勾選 `force_scope_display`，並取消 `Hide All Display Overlays`。可以拖曳面板或雙擊開成獨立視窗。更新世界檔後請重新載入 `worlds/Neura_Dual_Arm_Router_Service.wbt`，讓新增的感測器生效。

## Manual adjustment `task.ini`

| 區段 | 用途 |
| --- | --- |
| `[task]` | `run_order` 執行順序，以及各插孔的 `[x, y, z]` 世界座標 |
| `[calibration]` | 地面抓取、插入、裝好後抓取、放回地面的工具點與夾爪角度 |
| `[port_vision]` | 手臂一相機的 AOI 綠框 |
| `[adapter_vision]` | 手臂二相機的 AOI 綠框及相機安裝姿態 |

例如要先跑 A 再跑 B：

```ini
[task]
run_order = ["port_a", "port_b"]
port_a = [0.60, 0.08, 0.30]
port_b = [0.60, -0.08, 0.30]
```

修改後重設模擬。要新增 `port_c`，還需在 Webots 世界檔建立插孔及同名鎖扣 `port_c_latch`。新點位應先確認可達性、相機視野及避障路徑。`task.ini` 未列出的視覺門檻與控制值由 `settings.py` 提供預設；需要深入調整時可在相應 INI 區段加入同名設定覆蓋。

## 程式分工

| 檔案 | 負責內容 |
| --- | --- |
| `plan.py` | 依點位順序產生雙臂動作時程和工具座標 |
| `dual_arm_router_task.py` | 控制 Webots 裝置、夾持判斷、視覺校正與任務回報 |
| `port_vision.py`、`adapter_vision.py` | 各自辨識插孔與插座；未來可單獨更換為 AI 模型 |
| `vision_tracker.py` | 兩支相機共用的拍照、穩定判定、畫框和存圖 |
| `kinematics.py`、`vision_geometry.py` | 手臂逆運動學與相機投影計算 |
| `settings.py` | 讀取 INI 並提供內部預設值 |

相機影像會依點位儲存到 `.dual-arm-runtime`。手臂一會在插孔前停穩後拍照，再依定位結果插入；手臂二也會先停在插座前方取樣定位，確認穩定後才插入清潔頭。這可避免清潔頭太靠近時遮住橘色外框。辨識失敗時停止相應動作。目前辨識器使用影像顏色與幾何規則，尚未使用訓練過的 AI 模型。插座受接觸力與重力影響；插孔的 `Connector` 卡扣仍是理想化模型。

Port B 的拍照姿態與 Port A 一樣保持水平；B 點清潔頭的旋轉時間較長，以避開手腕接近奇異姿態時的急轉。完成後兩支手臂仍會按時程返回待命姿態。

離線檢查：`python -B -m unittest discover -s tools -p 'test_*.py'` 和 `python -B tools/verify_dual_arm_plan.py`。實際夾持、碰撞和相機畫面需在 Webots 內確認。
