'' Open_IMU
'' based on Madgwick Algorithm for MARG - uses gradient descent algo.

CON

  _clkmode      = xtal1 + pll16x
  _clkfreq      = 80_000_000

  SDA = 6
  SCL = 7

  COMPASS_XMIN = -345.0
  COMPASS_XMAX =  216.0
  COMPASS_YMIN = -347.0
  COMPASS_YMAX =  210.0
  COMPASS_ZMIN = -305.0
  COMPASS_ZMAX =  249.0

  INVERSE_XRANGE = 2.0 / (COMPASS_XMAX - COMPASS_XMIN)
  INVERSE_YRANGE = 2.0 / (COMPASS_YMAX - COMPASS_YMIN) 
  INVERSE_ZRANGE = 2.0 / (COMPASS_ZMAX - COMPASS_ZMIN)

  SAMPLE_FREQ = 512.0
  TWO_KP_DEF = 2.0 * 0.5
  TWO_KI_DEF = 2.0 * 0.00
  BETA_DEF = 0.08

  GYRO_SCALE = 0.07
                         
  PIBY2_FLOAT = 1.5707963
  PI_FLOAT = 3.14159265

  MILLI = _clkfreq / 1_000


VAR

  long rawAcclX, rawAcclY, rawAcclZ
  long rawMagX, rawMagY, rawMagZ
  long rawGyroX, rawGyroY, rawGyroZ
  
  long q0, q1, q2, q3, beta

  long magnitude

  long pitch, roll, yaw

  long gyroSumX, gyroSumY, gyroSumZ
  long offSetX, offSetY, offSetZ

  long floatMagX, floatMagY, floatMagZ
  long smoothAccX, smoothAccY, smoothAccZ
  long accToFilterX, accToFilterY, accToFilterZ
  
  
OBJ

  IMU         : "MinIMUv2-pasm"                '
  PST         : "Parallax Serial Terminal"
  fm          : "Float32Full"
  fs          : "FloatString"
    
PUB Init

  IMU.start(SCL,SDA)
  fm.start
  PST.start(250000)

  Main

PUB Main


PRI IMUinit

PRI IMUupdate | gx, gy, gz, azx, ay, az, recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3 

PRI AHRSupdate | gx, gy, gz, ax, ay, az, mx, my, mz, recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3

PRI GetEuler

PRI FastAtan2(y, x) | atan, z

PRI invSqrt(number) | i, x, y

  return(x ^ -0.5)
  
PRI Smoothing(raw, smooth)

  smooth := (raw * 0.15) + (smooth * 0.85)

  return(smooth)
                                                                     