# lio-sam-for-mulran

### MulRan dataset
- 아래에 따라 MulRan 데이터셋을 사용할 수 있습니다.
- You can also download [MulRan](https://sites.google.com/view/mulran-pr/home?authuser=0) dataset. (OS1-64)
- You need [File Player](https://github.com/irapkaist/file_player_mulran) for **MulRan dataset**. 
- This [guide video](https://www.youtube.com/watch?t=45&v=uU-FC-GmHXA&feature=youtu.be) makes it easy to learn how to use Mulran dataset.
  
### params.yaml 수정 부분

```yaml
lio_sam:

  # Topics
  pointCloudTopic: "/os1_points"
  imuTopic: "/imu/data_raw" 
  odomTopic: "odometry/imu" 
  gpsTopic: "/tcpfix"     

  # Sensor Settings
  sensor: ouster   
  N_SCAN: 64   
  Horizon_SCAN: 1024 
  downsampleRate: 1   
  lidarMinRange: 1.0
  lidarMaxRange: 120.0

  # IMU Settings
  imuAccNoise: 0.009939570888238808e-03
  imuGyrNoise: 0.005636343949698187e-03
  imuAccBiasN: 0.64356659353532566e-03
  imuGyrBiasN: 0.35640318696367613e-03
  imuGravity: 9.80511
  imuRPYWeight: 0.01

  # Extrinsics (lidar -> IMU)
  extrinsicTrans: [1.77, -0.00, -0.05]
  extrinsicRot: [-1, 0, 0,
                  0, -1, 0,
                  0, 0, 1]
  extrinsicRPY: [-1,  0, 0,
                 0, -1, 0,
                  0, 0, 1]
```

### imageProjection.cpp 수정 부분

- ousterpoint struct 수정 (17line)
   
```cpp
struct OusterPointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
     PCL_ADD_POINT4D;
     float intensity;
     uint32_t t;
     int ring;

     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 }EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT (OusterPointXYZIRT,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (uint32_t, t, t) (int, ring, ring)
)
```