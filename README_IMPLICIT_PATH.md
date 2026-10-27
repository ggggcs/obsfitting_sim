# 基于隐式函数的向量场导航

## 核心思想

与MATLAB代码一致，使用**隐式函数**直接定义路径，而不是通过路径点。这种方法更数学化、更优雅。

### 路径表示方式对比

| 方式 | 传统方法 | 隐式函数方法 |
|------|---------|-------------|
| **路径定义** | 一系列离散点 | 连续数学函数 |
| **梯度计算** | 从路径点插值 | 直接解析计算 |
| **向量场** | 依赖路径点查找 | 纯数学计算 |
| **精确度** | 受点密度限制 | 解析精确 |

## 圆形路径实现

### 数学定义

**隐式函数:**
```
φ(x,y) = (x - cx)² + (y - cy)² - R²
```

- 在圆上: φ = 0
- 圆外: φ > 0  
- 圆内: φ < 0

**梯度 (法向量):**
```
∇φ = [∂φ/∂x, ∂φ/∂y] = [2(x-cx), 2(y-cy)]
```

梯度指向圆外（径向向外）

### 向量场构造

**路径跟踪向量场 (论文公式3):**
```python
χ_P = direction_sign * E @ ∇φ - k_p * φ * ∇φ
```

其中:
- `E = [[0, -1], [1, 0]]` 旋转矩阵（逆时针90°）
- `E @ ∇φ` 产生切向分量（沿圆周）
- `k_p * φ * ∇φ` 产生法向分量（收敛到圆）
- `direction_sign` 控制顺时针/逆时针

**物理含义:**
1. **切向项** `direction_sign * E @ ∇φ`: 驱动沿圆周运动
2. **法向项** `-k_p * φ * ∇φ`: 驱动收敛到圆（当偏离时）

### 代码实现

```python
def circle_path_function(self, x, y):
    """圆形路径的隐式函数及梯度"""
    dx = x - self.circle_center_x
    dy = y - self.circle_center_y
    
    # 隐式函数值
    phi_val = dx**2 + dy**2 - self.circle_radius**2
    
    # 梯度: ∇φ = [2*dx, 2*dy]
    grad_phi = np.array([2*dx, 2*dy])
    
    return phi_val, grad_phi

def path_tracking_vector_field(self, x, y):
    """路径跟踪向量场"""
    phi, grad_phi = self.circle_path_function(x, y)
    
    # 方向: ccw (逆时针) = 1, cw (顺时针) = -1
    direction_sign = 1.0 if self.circle_direction == 'ccw' else -1.0
    
    # 向量场: χ_P = direction_sign * E * ∇φ - k_p * φ * ∇φ
    chi_P = direction_sign * (self.E @ grad_phi) - self.k_p * phi * grad_phi
    
    return chi_P
```

## 系统架构

```
┌──────────────────────────────┐
│   向量场规划器                │
│  (vector_field_planner.py)   │
│                              │
│  1. 圆形路径隐式函数          │
│     φ(x,y) = x² + y² - R²   │
│     ∇φ = [2x, 2y]           │
│                              │
│  2. 路径跟踪向量场 χ_P       │
│     = dir * E∇φ - k_p·φ·∇φ  │
│                              │
│  3. 障碍物反应向量场 χ_R     │
│     = E∇φ_i - k_r·φ_i·∇φ_i  │
│                              │
│  4. 复合向量场 χ_c           │
│     = (∏U_Q)χ̂_P + ∑(T_R·χ̂_R)│
└──────────────────────────────┘
           ↓
      发布 /cmd_vel
           ↓
┌──────────────────────────────┐
│      导航接口节点              │
│       (nav_sim.py)           │
│  转换为Ackermann命令          │
└──────────────────────────────┘
```

## 启动方法

```bash
roslaunch bringup implicit_path_navigation.launch
```

## 参数配置

编辑 `param/vector_field_planner_params.yaml`:

```yaml
# 路径定义
path_type: 'circle'
circle_center_x: 0.0    # 圆心X
circle_center_y: 0.0    # 圆心Y
circle_radius: 5.0      # 半径
circle_direction: 'ccw' # 方向

# 向量场增益
k_p: 1.0  # 路径跟踪增益（越大收敛越快）
k_r: 1.5  # 反应向量场增益（越大避障越激进）
```

## 与MATLAB代码的对应

| MATLAB | Python实现 | 说明 |
|--------|-----------|------|
| `phi = @(x, y) x.^2 + y.^2 - R^2` | `circle_path_function()` | 圆形隐式函数 |
| `grad_phi_x = @(x, y) 2*x` | `grad_phi[0] = 2*dx` | X方向梯度 |
| `grad_phi_y = @(x, y) 2*y` | `grad_phi[1] = 2*dy` | Y方向梯度 |
| `chi_P = ...` | `path_tracking_vector_field()` | 路径跟踪场 |
| `E = [0, -1; 1, 0]` | `self.E` | 旋转矩阵 |

## 扩展到其他路径

### 直线路径
```python
def line_path_function(self, x, y):
    """直线路径: ax + by + d = 0"""
    phi_val = self.a*x + self.b*y + self.d
    grad_phi = np.array([self.a, self.b])
    return phi_val, grad_phi
```

### 8字形路径
```python
def figure8_path_function(self, x, y):
    """8字形 (莱姆尼斯卡特曲线)"""
    a = self.figure8_size
    phi_val = (x**2 + y**2)**2 - 2*a**2*(x**2 - y**2)
    
    grad_x = 4*x*(x**2 + y**2) - 4*a**2*x
    grad_y = 4*y*(x**2 + y**2) + 4*a**2*y
    grad_phi = np.array([grad_x, grad_y])
    
    return phi_val, grad_phi
```

## 优势

1. **数学优雅**: 直接使用隐式函数，符合理论推导
2. **精确计算**: 解析梯度，无插值误差
3. **无需规划器**: 不需要A*或路径发布器
4. **易于扩展**: 添加新路径只需定义新的隐式函数
5. **与论文一致**: 完全按照向量场论文的数学形式实现

## 调试技巧

### 查看向量场
```bash
# 在RViz中查看向量场可视化
# 话题: /vector_field_planner/vector_field
```

### 检查路径误差
```python
# 在圆上: |φ| ≈ 0
# 圆外: φ > 0
# 圆内: φ < 0
rostopic echo /vector_field_planner/path_error
```

### 调整参数
- **k_p↑**: 更快收敛到路径，可能震荡
- **k_p↓**: 更平滑，收敛慢
- **radius↑**: 更大的圆
- **direction**: 切换顺/逆时针

## 故障排除

1. **小车不动**: 检查隐式函数是否返回有效梯度
2. **偏离路径**: 增加 `k_p` 值
3. **震荡**: 减小 `k_p` 或增加阻尼
4. **方向错误**: 切换 `circle_direction`

## 性能对比

| 指标 | 路径点方法 | 隐式函数方法 |
|------|-----------|-------------|
| 计算复杂度 | O(n) 路径点 | O(1) 常数 |
| 梯度精度 | 插值近似 | 解析精确 |
| 内存占用 | 存储路径点 | 仅存储参数 |
| 扩展性 | 需要重新规划 | 修改函数 |

这种方法完全符合你的论文和MATLAB代码的数学实现！
