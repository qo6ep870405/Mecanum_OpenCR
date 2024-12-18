# 麥克納姆輪全向移動機器人<br>Mecanum-wheeled omnidirectional mobile robot
此project使用ROBOTICS公司的嵌入式微控制器開發板OpenCR1.0及使用其公司智能伺服馬達Dynamixel XH540-W150-R，此外使用上位單板電腦NVIDIA Jetson™ TX2實現ROS整合開發。
## 硬體需求
1.OpenCR1.0，負責逆向運動學計算，接收編碼器資訊更新機器人姿態。  
2.Dynamixel XH540-W150-R x 4，伺服馬達。  
3.麥克納姆輪(Mecanum wheel) x 4，萬向輪。  
4.NVIDIA Jetson™ TX2，單板電腦，上位命令。  
## 程式說明

本專案的文件目錄如下：  
- **Mecanum.ino**: 主程序，負責系統初始化與執行主邏輯。
- **mecanum.cpp & mecanum.h**: 定義麥克納姆輪運動控制的核心功能，包括逆向運動學與編碼器計算。
- **mecanum_ros.h**: ROS通訊定義，用於上位機命令與狀態傳輸。
- **motor_driver.cpp & motor_driver.h**: 馬達速度寫入與編碼器位置讀取。


1.逆向運動學

<div align=center>

![幾何模型](/figures/simplified_geometry_of_mobile_robot.png)

圖一、機器人坐標系下簡化幾何運動學模型
<div align=left>
如圖所示，其中 

- $l_x$ 代表前輪中心點至後輪中心點之一半長度， $l_y$ 代表左右兩輪距離之一半長度 $r$ 為輪半徑。  
- $v_x,v_y$ 分別對應機器人延 $X_R,Y_R$ 方向上的線速度，單位為 $(m/s)$ ， $ω_z$ 為機器人繞 $Z_R$ 軸方向旋轉之角速度，單位為 $(rad/s)$ 。

機器人的逆向運動學在給定 $v_x,v_y$ 及 $ω_z$ 下，求得各輪之線性速度為:

$$
\left\lbrace\begin{matrix}
v_1 = v_x-v_y-(l_x+l_y)\omega _z \\
v_2 = v_x+v_y+(l_x+l_y)\omega _z \\
v_3 = v_x+v_y-(l_x+l_y)\omega _z \\
v_4 = v_x-v_y+(l_x+l_y)\omega _z \\
\end{matrix}\right.
$$

再由 $ω=\frac{v}{r}$ ，推算每個馬達的轉速，詳細見void Mecanum::controlMecanum(void)中的實現。

2.編碼器與里程計

$$
\begin{bmatrix}
x(k) \\
y(k) \\
θ(k)
\end{bmatrix}
\=
\begin{bmatrix}
x(k-1) + \frac{r\sqrt{2}}{4} \left( \sin\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{1w} + \cos\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{2w} + \cos\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{3w} + \sin\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{4w} \right) \\
y(k-1) + \frac{r\sqrt{2}}{4} \left( -\cos\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{1w} + \sin\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{2w} + \sin\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{3w} - \cos\left(\tilde{θ}(k-1) + \frac{\Deltaθ(k)}{2} \right) \Deltaθ_{4w} \right) \\
θ(k-1) + \frac{r}{4(l_x + l_y)} \left( -\Deltaθ_{1w} + \Deltaθ_{2w} - \Deltaθ_{3w} + \Deltaθ_{4w} \right)
\end{bmatrix}\
$$

其中 $\Deltaθ(k)=\frac{r}{4(l_x + l_y)} \left( -\Deltaθ_{1w} + \Deltaθ_{2w} - \Deltaθ_{3w} + \Deltaθ_{4w} \right)$ ， $\tilde{θ}(k-1)=θ(k-1)+\frac{π}{4}$ 

此處實現參照void Mecanum::updatePose(void)，首先讀取各個馬達當前時間 $k$ 編碼器刻度，計算與上一時刻 $k-1$ 之間的差距，利用TICK2RAD將刻度差轉為徑度差，隨後利用上述公式更新機器人當前姿態。  

3.ROS機器人作業系統

從上層單板電腦Subscribe速度命令topic(/cmd_vel)及清除命令(/cmd_clear)，用以歸零里程計速度命令資訊，OpenCR這端Publish里程計topic(/odom)，以及TF轉換。

## 安裝與設定
1.馬達設定:首先使用RoboPlus Manager 2.0設定每顆馬達的Baud rate,ID及將右半部(右前.右後)馬達改為reverse mode(詳細Baud rate, ID設定值參考motor_driver.h)。  
2.馬達連接:使用菊鍊式(Daisy chain)方式串接所有馬達至OpenCR板TTL接口上。  
3.程式燒錄:使用Arduino IDE將此程式燒錄製OpenCR板上，如何新增OpenCR開發板至Arduino IDE詳情至ROBOTICS官網翻閱。  