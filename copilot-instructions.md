# LVINS C++ 编程规范

你是一名严格遵循 LVINS 项目编程规范的 C++ 代码生成助手。在编写或审查代码时，请**严格遵守**以下所有规范。

> **版本**：v1.0 · **维护**：在文件末尾的「十二、补充规范」章节中追加新约定，不得修改已有章节编号。

---

## 一、文件结构规范

1. **头文件保护**：所有头文件一律使用 `#pragma once`，禁止使用 `#ifndef` 宏守卫。

2. **include 顺序**（两段式，段间空行分隔）：
    - 第一段：项目内部头文件，使用双引号，路径以 `lvins_xxx/` 开头
    - 第二段：第三方库头文件，使用尖括号
   ```cpp
   #include "lvins_common/eigen_types.h"
   #include "lvins_common/logger.h"

   #include <Eigen/Core>
   #include <sophus/se3.hpp>
   #include <memory>
   #include <vector>
   ```

3. **模板实现文件**：模板类的实现放在 `impl/xxx.hpp` 中，并在对应头文件末尾通过 `#include` 引入，不得将实现直接写在头文件类体内（内联短函数除外）。

4. **命名空间闭合注释**：命名空间右花括号后须添加注释：
   ```cpp
   } // namespace lvins
   } // namespace lvins::rotation_helper
   ```

---

## 二、命名规范

### 2.1 命名空间

- 全小写 `snake_case`，顶层为 `lvins`，子命名空间形如 `lvins::module_name`。
- 工具函数命名空间示例：`namespace lvins::rotation_helper`

### 2.2 类与结构体

- **PascalCase**（大驼峰），语义清晰，名词或名词短语：
  ```cpp
  class AsyncQueue;
  class IncrementalVoxelMap;
  struct NavState;
  struct VoxelInfo;
  ```
- 纯接口/抽象基类以 `Base` 结尾：`FrameBase`、`LandmarkBase`、`RobustKernelBase`
- 不可继承的终态类加 `final`：`class NullKernel final`、`class PoseManifold final`

### 2.3 模板参数

- 类型参数：`PascalCase`，语义明确：`typename Derived`、`typename VoxelContent`、`typename Landmarks`
- 标量类型参数：使用 `Scalar_`（带尾部下划线）：`template<class Scalar_>`

### 2.4 函数与方法

- **camelCase**（小驼峰）：
  ```cpp
  void addTimeTask(const TimeTask::Ptr &task);
  [[nodiscard]] int64_t timestamp() const;
  [[nodiscard]] long bundleId() const;
  ```
- **getter**：直接以属性名命名，无 `get` 前缀：`timestamp()`、`id()`、`scales()`
- **setter**：`set` + PascalCase 属性名：`setBundleId()`、`setTwf()`、`setCapacity()`
- 工具命名空间内的自由函数同样 camelCase：`getCpuSerialNumber()`、`reduce()`

### 2.5 成员变量

- **私有/保护成员**：`snake_case_`（带尾部下划线）：
  ```cpp
  int64_t timestamp_;
  long bundle_id_{-1};
  size_t lru_counter_{0};
  ```
- **公有数据成员**（`struct` 中）：`snake_case`（无下划线）：
  ```cpp
  int64_t timestamp;
  SE3F T;
  Vec3F vel;
  ```
- **静态成员变量**：同样遵循尾部下划线规则：
  ```cpp
  inline static std::atomic<long> id_counter_{0};
  ```
- **坐标变换命名约定**：`T_ab` 表示从 b 系到 a 系的变换（即目标系在前，源系在后），成员变量加下划线后缀：`T_wf_`（从帧系到世界系）、
  `T_bs_`（从传感器系到机体系），函数参数对应为 `T_wf`、`T_bs`（无尾部下划线）。

### 2.6 函数参数与局部变量

- `snake_case`，无尾部下划线：`timestamp`、`bundle_id`、`file_name`、`wait_time`

### 2.7 宏

- 全大写 + `LVINS_` 前缀：`LVINS_TRACE`、`LVINS_CHECK`、`LVINS_FORMAT`

---

## 三、注释规范

### 3.1 文档注释（Doxygen 风格）

所有类、结构体、公有/保护方法、公有成员变量**必须**有 Doxygen 文档注释，格式如下：

```cpp
/**
 * @brief 简短描述（一句话）
 * @details 详细描述（可选，多句话）
 * @tparam Derived 派生类类型
 * @param timestamp 帧时间戳（ns）
 * @param T_wf 帧位姿
 * @return 帧时间戳
 * @note 附加说明
 * @warning 使用警告
 */
```

- `@brief` 使用中文，简洁明了，句末不加句号
- `@param` 和 `@return` 说明中注明单位（如适用）
- `@warning` 用于描述使用前置条件或危险行为
- `@note` 用于说明引用来源、外部链接或补充事项

### 3.2 成员变量行内注释

使用 `///<` 格式，紧跟在成员声明之后，注明语义和单位：

```cpp
int64_t timestamp_;  ///< 时间戳（ns）
long bundle_id_{-1}; ///< 帧束id（用于多同类型帧传感器，历史唯一）
Float leaf_size_;    ///< 体素尺寸
```

### 3.3 代码段注释

- 单行注释使用 `//`，注意与代码保持一个空格：`// 清空队列`
- 模块分隔、特殊逻辑或引用说明使用 `/// @note` 或块注释

---

## 四、类设计规范

### 4.1 禁止拷贝

需要禁止拷贝的类，通过继承 `NonCopyable` 实现，不要手动写 `= delete`：

```cpp
class MyClass : public NonCopyable { ... };
```

### 4.2 智能指针别名

在类体内统一定义常用指针类型别名：

```cpp
using Ptr      = std::shared_ptr<ClassName>;
using ConstPtr = std::shared_ptr<const ClassName>;
// 或
using Ptr = std::unique_ptr<ClassName>;
```

### 4.3 集合类型别名

为常用集合类型在命名空间级别定义别名：

```cpp
using NavStates = std::vector<NavState, Alloc<NavState>>;
```

### 4.4 构造/析构规范

- 仅有一个参数的构造函数必须加 `explicit`
- 平凡析构函数统一写 `~ClassName() = default;` 并附带 `@brief 默认析构函数` 注释
- 显式删除的函数（拷贝等）使用 `= delete`

### 4.5 访问控制顺序

类体内按以下顺序组织：`public` → `protected` → `private`

### 4.6 getter / setter 声明顺序

同一属性的 getter 必须声明在对应 setter **之前**：

```cpp
[[nodiscard]] const SE3F &Twf() const;  // getter 在前
void setTwf(const SE3F &T_wf);          // setter 在后
void setTwf(SE3F &&T_wf);               // 移动版 setter 紧随其后
```

---

## 五、函数属性规范

- 所有具有返回值、且调用方**不应忽略**返回值的函数，加 `[[nodiscard]]`（getter、查询函数、工厂函数等）
- 虚函数重写加 `override`，终态类加 `final`
- 不修改成员状态的成员函数加 `const`

---

## 六、初始化规范

- 成员变量在声明处使用花括号初始化：
  ```cpp
  long bundle_id_{-1};
  bool outlier_{false};
  size_t lru_counter_{0};
  ```
- 避免不必要的默认值（由构造函数参数赋值的成员无需在声明处初始化）

---

## 七、格式化器规范

每个需要格式化输出的类型，都需要在文件末尾（**命名空间外**）提供 `LVINS_FORMATTER` 模板特化，包含静态 `parse` 和 `format`
方法，实现均委托给类的 `print()` 方法：

```cpp
template<>
struct LVINS_FORMATTER<lvins::MyType> {
    /**
     * @brief 从文本中解析格式化字符
     * @param ctx 文本
     * @return 格式化字符尾部迭代器
     */
    static constexpr auto parse(const LVINS_FORMAT_PARSE_CONTEXT &ctx) {
        return ctx.begin();
    }

    /**
     * @brief 格式化
     * @param obj 对象
     * @param ctx 输出的格式化文本
     * @return 输出格式化文本的尾部迭代器
     */
    static auto format(const lvins::MyType &obj, LVINS_FORMAT_CONTEXT &ctx) {
        return LVINS_FORMAT_TO(ctx.out(), "{}", obj.print());
    }
};
```

对于有继承体系的类型，使用 `std::enable_if_t` + `std::is_base_of_v` 做模板偏特化。

---

## 八、日志规范

- 使用项目宏记录日志，**禁止**使用 `std::cout` 或 `printf`：
  ```cpp
  LVINS_TRACE(...)   // 追踪级别
  LVINS_DEBUG(...)   // 调试级别
  LVINS_INFO(...)    // 信息级别
  LVINS_WARN(...)    // 警告级别
  LVINS_ERROR(...)   // 错误级别
  LVINS_FATAL(...)   // 致命错误（自动调用 abort）
  ```
- 断言检查使用 `LVINS_CHECK(condition, "格式化消息 {}", param)`

---

## 九、中英文混排规范

中文注释中出现英文标识符、缩写词、专有名词时，遵循以下统一用词表，**禁止**在同一语境下混用大小写或中英文不同写法。

### 9.1 专有名词与缩写统一表

| 概念              | 规范写法                 | 禁止写法示例                      |
|-----------------|----------------------|-----------------------------|
| 标识符（数据库/对象唯一编号） | `id`                 | `ID`、`Id`                   |
| 最近最少使用算法        | `LRU`                | `lru`、`Lru`                 |
| 惯性测量单元          | `IMU`                | `imu`、`Imu`                 |
| 激光雷达            | `雷达` 或 `lidar`       | `LIDAR`、`LiDAR`（正文注释中）      |
| 相机              | `相机` 或 `camera`      | `Camera`（正文注释中）             |
| 里程计             | `里程计` 或 `odom`       | `odometry`（注释简称中）           |
| YAML 配置文件/节点    | `YAML`               | `yaml`、`Yaml`               |
| 旋转李群/李代数        | `SO3`、`SE3`          | `so3`、`se3`、`So3`           |
| 四元数             | `四元数`                | `quaternion`（中文注释中）         |
| 协方差矩阵           | `协方差矩阵` 或 `cov`（变量名） | `covariance`（中文注释变量名中）      |
| 雅可比矩阵           | `雅可比矩阵`              | `Jacobian`（中文注释中）           |
| 陀螺仪             | `陀螺仪`                | `gyroscope`（中文注释中）          |
| 加速度计            | `加速度计`               | `accelerometer`（中文注释中）      |
| 零偏              | `零偏`                 | `偏置`、`bias`（中文注释中）          |
| 外参              | `外参`                 | `外参数`、`extrinsic`（中文注释中）    |
| 时间戳             | `时间戳`                | `时间戳记`、`timestamp`（中文注释中）   |
| 位姿              | `位姿`                 | `姿态`（含位置+姿态时）、`pose`（中文注释中） |
| 路标              | `路标`                 | `landmark`（中文注释中）           |
| 体素              | `体素`                 | `voxel`（中文注释中）              |
| 点云              | `点云`                 | `point cloud`（中文注释中）        |
| 帧               | `帧`                  | `frame`（中文注释中）              |
| 帧束              | `帧束`                 | `frame bundle`（中文注释中）       |
| 粗差              | `粗差`                 | `外点`、`outlier`（中文注释中）       |

### 9.2 中英文混排格式

- 中文与英文/数字之间须插入一个空格：`获取帧 id`、`时间戳（ns）`、`LRU 删除阈值`
- 注释中的括号一律使用中文括号 `（）`
- 单位说明统一写在括号内，紧跟参数描述，直接填写单位符号，无需加「单位：」前缀：`时间戳（ns）`、`等待时间（ns）`、
  `陀螺仪数据（rad/s）`、`加速度计数据（m/s^2）`
- URL、路径等技术字符串原样保留，不加空格

### 9.3 布尔返回值注释句式

返回 `bool` 的函数，`@return` 统一使用「……是否……」句式：

```cpp
* @return 队列是否为空
* @return IMU 数据是否已初始化
* @return 是否操作成功
* @return 是否反序列化成功
```

### 9.4 getter / setter 注释句式

- getter `@brief` 统一为：`获取 <属性名>`；`@return` 为：`<属性名>`
- setter `@brief` 统一为：`设置 <属性名>`；`@param` 为：`<参数名> <属性名>`

```cpp
* @brief 获取帧时间戳（ns）
* @return 帧时间戳

* @brief 设置帧位姿
* @param T_wf 帧位姿
```

---

## 十、杂项规范

- **Eigen 内存对齐**：需要用到 Eigen 固定尺寸对象的容器，使用 `Alloc<T>` 分配器：`std::vector<NavState, Alloc<NavState>>`
- **`clang-format` 关闭区域**：对 PCL 宏注册等无法自动格式化的代码块，使用 `// clang-format off` / `// clang-format on` 包裹

---

## 十一、英文命名惯用词规范

中文注释之外，代码中的变量名、参数名、函数名同样存在惯用缩写与全称的统一要求。

### 11.1 常用缩写统一表

| 概念         | 规范写法          | 禁止写法示例                             |
|------------|---------------|------------------------------------|
| 陀螺仪        | `gyr`         | `gyro`、`gyroscope`                 |
| 加速度计       | `acc`         | `accel`、`accelerometer`            |
| 陀螺仪零偏      | `bg`          | `bias_gyr`、`gyr_bias`、`gyro_bias`  |
| 加速度计零偏     | `ba`          | `bias_acc`、`acc_bias`、`accel_bias` |
| 速度         | `vel`         | `v`、`velocity`                     |
| 位姿（SE3 变换） | `T`           | `pose`、`transform`（变量名中）           |
| 旋转部分       | `rot`         | `rotation`、`R`                     |
| 平移部分       | `trans`       | `translation`、`t`                  |
| 协方差        | `cov`         | `covariance`（变量名中）                 |
| 标准差后缀      | `_std`        | `_sigma`、`_dev`                    |
| 倒数/逆       | `inv_` 前缀     | `reciprocal_`、`inverse_`           |
| 配置节点       | `config`      | `cfg`、`yaml_node`、`conf`           |
| 回调函数       | `callback`    | `cb`、`func`                        |
| 点云         | `point_cloud` | `pc`、`pointcloud`、`cloud`          |
| 文件名        | `file_name`   | `filename`、`fname`                 |
| 文件路径       | `file_path`   | `filepath`、`path`（有歧义时）            |
| 索引         | `index`       | `idx`、`i`（非循环变量时）                  |
| 坐标         | `coord`       | `coordinate`、`pos`（体素坐标时）          |
| 时间戳        | `timestamp`   | `ts`、`time`、`stamp`                |
| 标识符        | `id`          | `ID`、`Id`、`identifier`             |

### 11.2 坐标系字母约定

变换矩阵命名为 `T_<目标系><源系>`，坐标系字母如下：

| 字母  | 坐标系             |
|-----|-----------------|
| `w` | 世界系（world）      |
| `b` | 机体系（body）       |
| `f` | 帧坐标系（frame）     |
| `s` | 传感器系（sensor）    |
| `l` | 激光雷达系（lidar）    |
| `c` | 相机系（camera）     |
| `h` | 水平系（horizontal） |

示例：`T_wf`（从帧系到世界系）、`T_bs`（从传感器系到机体系）

### 11.3 函数名动词前缀约定

| 场景                | 前缀                  | 示例                                    |
|-------------------|---------------------|---------------------------------------|
| 简单 getter（直接返回成员） | 无前缀                 | `timestamp()`、`id()`、`scales()`       |
| 需计算/查找的 getter    | `get`               | `getCellIndex()`、`getLandmarkPoint()` |
| setter            | `set`               | `setTwf()`、`setCapacity()`            |
| 布尔状态查询            | `is`                | `isOutlier()`、`isPeriodic()`          |
| 非阻塞尝试操作           | `try`               | `tryPop()`                            |
| 序列化写出             | `writeToYaml`       | —                                     |
| 布尔标志成员变量          | `is_` 前缀或 `use_` 前缀 | `is_periodic_`、`use_lidar_`           |

---

## 十二、补充规范

> 本章节用于记录后续新增的约定。请按序编号追加，不得删改已有条目。

<!-- 示例格式：
### 12.1 <标题>
<内容>
-->

