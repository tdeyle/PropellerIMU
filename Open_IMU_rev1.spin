'' Open_IMU
'' based on Madgwick Algorithm for MARG - uses gradient descent algo.


''Todo: Remove the floating types and replace raw values with integers.


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

  long int_rawAcctX, int_rawAcctY, int_rawAcctZ
  long int_rawMagX, int_rawMagY, int_rawMagZ
  long int_rawGyroX, int_rawGyroY, int_rawGyroZ
  
  long q0, q1, q2, q3, beta

  long r11, r12, r13
  long r21, r22, r23
  long r31, r32, r33

  long magnitude

  long fp_pitch, fp_roll, fp_yaw

  long int_gyroX, int_gyroY, int_gyroZ
  long fp_offSetX, fp_offSetY, fp_offSetZ

  long floatMagX, floatMagY, floatMagZ
  long fSmoothAccX, fSmoothAccY, fSmoothAccZ
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

  IMUinit
   

PRI IMUinit | i, int_acclXX, int_acclYY, int_acclZZ, int_negAcclY, int_l, fp_l, fp_m, fp_n, fp_j, fp_negj, fp_k, q_denom, sinPitch, cosPitch, sinRoll, cosRoll, sinYaw, cosYaw, fp_xmag, fp_ymag
                   
  int_gyroX := int_gyroY := int_gyroZ := 0
   
  repeat i from 0 to 500
    'This rawGyro group is NOT floating, since we are doing integer math below for the average.
    PST.dec(i)
    int_rawGyroX := IMU.getRx
    int_rawGyroY := IMU.getRy
    int_rawGyroZ := IMU.getRz

    printData(0, 0, int_rawGyroX)
    printData(0, 1, int_rawGyroY)
    printData(0, 2, int_rawGyroZ)
     
    int_rawAcctX := IMU.getAx
    int_rawAcctY := IMU.getAy
    int_rawAcctZ := IMU.getAz

    PST.position(0,3)
    PST.str(string("Accelerometer Readings: "))
    'printData(0, 4, int_rawAcctX)
    'printData(0, 5, int_rawAcctY)
    'printData(0, 6, int_rawAcctZ)

    printDataFP(0, 4, fm.ffloat(int_rawAcctX))
    printDataFP(0, 5, fm.ffloat(int_rawAcctY))
    printDataFP(0, 6, fm.ffloat(int_rawAcctZ))
     
    int_rawMagX := IMU.getMx
    int_rawMagY := IMU.getMy
    int_rawMagZ := IMU.getMz   

    PST.position(0,7)
    PST.str(string("Compass Readings: "))
    printData(0, 8, int_rawMagX)
    printData(0, 9, int_rawMagY)
    printData(0, 10, int_rawMagZ)
     
    fSmoothAccX := Smoothing(fm.FFloat(int_rawAcctX), fSmoothAccX)
    fSmoothAccY := Smoothing(fm.FFloat(int_rawAcctY), fSmoothAccY)
    fSmoothAccZ := Smoothing(fm.FFloat(int_rawAcctZ), fSmoothAccZ)

    PST.position(0, 23)
    PST.str(string("Smoothed Accel: "))
    printDataFP(0, 24, fSmoothAccX)
    printDataFP(0, 25, fSmoothAccY)
    printDataFP(0, 26, fSmoothAccZ)
                                  
    int_gyroX += int_rawGyroX
    int_gyroY += int_rawGyroY
    int_gyroZ += int_rawGyroZ
                   
    PST.position(0,11)
    PST.str(string("Gyro Summed Readings: "))
    printData(0, 12, int_gyroX)
    printData(0, 13, int_gyroY)
    printData(0, 14, int_gyroZ)

    waitcnt(clkfreq/1_000 * 3 + cnt)
   
  fp_offSetX := fm.FDiv(fm.FFloat(int_gyroX), 500.0)
  fp_offSetY := fm.FDiv(fm.FFloat(int_gyroY), 500.0)
  fp_offSetZ := fm.FDiv(fm.FFloat(int_gyroZ), 500.0)
                                    
  PST.position(0,15)
  PST.str(string("Gyro OffSets: "))
  printDataFP(0, 16, fp_offSetX)
  printDataFP(0, 17, fp_offSetY)
  printDataFP(0, 18, fp_offSetZ)

  'Calculate initial Quaternion
  'pitch := fm.Degrees(fastAtan2(int_rawAcctX, sqrt(int_rawAcctY * int_rawAcctY + int_rawAcctZ * int_rawAcctZ)))
  'roll := fm.Degrees(fastAtan2(-1 * int_rawAcctY, sqrt(int_rawAcctX * int_rawAcctX + int_rawAcctZ * int_rawAcctZ)))
  int_acclXX := int_rawAcctX * int_rawAcctX
  int_acclYY := int_rawAcctY * int_rawAcctY
  int_acclZZ := int_rawAcctZ * int_rawAcctZ

  PST.position(0,19)
  PST.str(string("Accel Squared: "))
  printData(0, 20, int_acclXX)
  printData(0, 21, int_acclYY)
  printData(0, 22, int_acclZZ)

  int_negAcclY := -int_rawAcctY
   
  int_l := int_acclYY + int_acclZZ
  fp_m := fm.FSqr(fm.FFloat(int_l))
  fp_n := fastAtan2(fm.FFloat(int_rawAcctX), fp_m)
   
  fp_pitch := fm.Degrees(fp_n)

  PST.position(0, 27)
  printDataFP(0, 28, fp_pitch)
   
  int_l := int_acclXX + int_acclZZ
  fp_m := fm.FSqr(fm.FFloat(int_l))
  fp_n := fastAtan2(fm.FFloat(int_negAcclY), fp_m)
  
  fp_roll := fm.Degrees(fp_n)

  printDataFP(0, 29, fp_roll)
   
  if int_rawAcctZ > 0
    if int_rawAcctX > 0
      fp_pitch := fm.FSub(180.0, fp_pitch)
   
    else
      fp_pitch := fm.FSub(-180.0, fp_pitch)
   
    if int_rawAcctY > 0
      fp_roll := fm.FSub(-180.0, fp_roll)
   
    else
      fp_roll := fm.FSub(180.0, fp_roll)
   
   
   
  floatMagX := fm.FSub(fm.FMul(fm.FSub(fm.FFloat(int_rawMagX), COMPASS_XMIN), INVERSE_XRANGE), 1.0)
  floatMagY := fm.FSub(fm.FMul(fm.FSub(fm.FFloat(int_rawMagY), COMPASS_YMIN), INVERSE_YRANGE), 1.0)
  floatMagZ := fm.FSub(fm.FMul(fm.FSub(fm.FFloat(int_rawMagZ), COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)
   
  cosPitch := fm.Cos(fm.Radians(fp_pitch))
  sinPitch := fm.Sin(fm.Radians(fp_pitch))
  cosRoll := fm.Cos(fm.Radians(fp_roll))
  sinRoll := fm.Sin(fm.Radians(fp_roll))
   
  'xMag := (floatMagX * cos(fm.Radians(pitch))) + floatMagZ * sin(fm.Radians(pitch)))
  fp_j := fm.FMul(cosPitch, floatMagX)
  fp_k := fm.FMul(sinPitch, floatMagZ)
  fp_xMag := fm.FAdd(fp_j, fp_k)
   
  'yMag := -1 * ((floatMagX * sin(fm.Radians(roll)) * sin(fm.Radians(pitch))) + (floatMagY * cos(fm.Radians(roll))) - (floatMagZ * sin(fm.Radians(roll)) * cos(fm.Radians(pitch))))
  fp_j := fm.FMul(fm.FMul(floatMagX, sinRoll), sinPitch)
  fp_negJ := fm.FNeg(fp_j)
  fp_k := fm.FMul(floatMagY, cosRoll)
  fp_l := fm.FMul(fm.FMul(floatMagZ, sinRoll), cosPitch)
  fp_yMag := fm.FSub(fm.FAdd(fp_negJ, fp_k), fp_l)
   
  fp_yaw := fm.Degrees(fastAtan2(fp_yMag, fp_xMag))
   
  if fp_yaw < 0.0
    fp_yaw := fm.FAdd(fp_yaw, 360.0)
   
  cosYaw := fm.Cos(fm.Radians(fp_yaw))
  sinYaw := fm.Sin(fm.Radians(fp_yaw))
   
  r11 := fm.FMul(cosPitch, cosYaw)
  r21 := fm.FMul(cosPitch, sinYaw)
  r31 := fm.FNeg(sinPitch)
   
  r12 := fm.FAdd(fm.FNeg(fm.FMul(cosRoll, sinYaw)), fm.FMul(fm.FMul(sinRoll, sinPitch), cosYaw))
  r22 := fm.FAdd(fm.FMul(cosRoll, cosYaw), fm.FMul(fm.FMul(sinRoll, sinPitch), sinYaw))
  r23 := fm.FMul(sinRoll, cosPitch)
   
  r31 := fm.FAdd(fm.FMul(sinRoll, sinYaw), fm.FMul(fm.FMul(cosRoll, sinPitch), cosYaw))
  r32 := fm.FAdd(fm.FNeg(fm.FMul(sinRoll, cosYaw)), fm.FMul(fm.FMul(cosRoll, sinPitch), sinYaw))
  r33 := fm.FMul(cosRoll, cosPitch)
   
   
  q0 := fm.FDiv(0.5, fm.FSqr(fm.FAdd(fm.FAdd(1.0, r11), fm.FAdd(r22, r33))))
  q_denom := fm.FMul(4.0, q0)
  q1 := fm.FDiv(fm.FSub(r32, r23), q_denom)
  q2 := fm.FDiv(fm.FSub(r13, r31), q_denom)
  q3 := fm.FDiv(fm.FSub(r21, r12), q_denom)

  PST.position(30,0)
  PST.str(string("Quaternions: "))
  printDataFP(30, 1, q0)
  printDataFP(30, 2, q1)
  printDataFP(30, 3, q2)
  printDataFP(30, 4, q3)

  PST.position(30, 5)
  PST.str(string("Location: "))
  printDataFP(30, 6, fp_pitch)
  printDataFP(30, 7, fp_roll)
  printDataFP(30, 8, fp_yaw)
  
    
PRI IMUupdate | gx, gy, gz, ax, ay, az, recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3 

PRI AHRSupdate | gx, gy, gz, ax, ay, az, mx, my, mz, recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4, _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3

PRI GetEuler

PRI FastAtan2(y, x) | atan, z, j, k, l
  
  if x == 0.0
    if y > 0.0
      return PIBY2_FLOAT
    if y == 0.0
      return 0.0
    return -PIBY2_FLOAT
  
  z = fm.FDiv(y, x)
  if fm.FAbs(z) < 1.0
  
    atan := fm.FDiv(z, fm.FAdd(1.0, fm.FMul(0.28, fm.FMul(z, z))))
    if x < 0.0
    
      if y < 0.0
        return fm.FSub(atan, PI_FLOAT)
      return fm.FAdd(atan, PI_FLOAT)
  
  else
  
    atan := fm.FSub(PIBY2_FLOAT, fm.FDiv(z, fm.FAdd(fm.FMul(z, z), 0.28)))
    if y < 0.0
      return fm.FSu(atan, PI_FLOAT)
  
  return atan
    
   
PRI invSqrt(number) | i, x, y
    
  return(fm.Pow(x, -0.5))
    
PRI Smoothing(raw, smooth)

  return(fm.FAdd(fm.FMul(raw, 0.15), FM.FMul(smooth, 0.85)))

PRI ToDeg(x) | i

  return(fm.FMul(x, 57.2957795131))

PRI ToRad(x)

  return(fm.FMul(x, 0.01745329252))

PRI printDataFP(col, row, data)

  PST.position(col, row)
  PST.str(fs.floattostring(data))
  PST.ClearEnd

PRI printData(col, row, data)

  PST.position(col, row)
  PST.dec(data)
  PST.ClearEnd
  