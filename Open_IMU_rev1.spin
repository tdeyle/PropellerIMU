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
  TWO_KP_DEF = 2.0 * 0.05
  TWO_KI_DEF = 2.0 * 0.00

  GYRO_SCALE = 0.07
                         
  PIBY2_FLOAT = 1.5707963
  PI_FLOAT = 3.14159265

  MILLI = _clkfreq / 1_000


VAR

  
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

