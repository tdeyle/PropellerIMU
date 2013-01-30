'' Open_IMU
'' based on Madgwick Algorithm for MARG

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
  TWO_KI_DEF = 2.0 * 0.0

  GYRO_SCALE = 0.07
                         
  PIBY2_FLOAT = 1.5707963
  PI_FLOAT = 3.14159265

  MILLI = _clkfreq / 1_000


VAR

  long rawAcclX, rawAcclY, rawAcclZ
  long rawMagX, rawMagY, rawMagZ
  long rawGyroX, rawGyroY, rawGyroZ
  
  long q0, q1, q2, q3

  long r11, r12, r13
  long r21, r22, r23
  long r31, r32, r33
  
  long beta
  long magnitude
  long pitch, roll, yaw

  long sinPitch, cosPitch
  long sinRoll, cosRoll
  long sinYaw, cosYaw

  long gyroSumX, gyroSumY, gyroSumZ
  long offsetX, offsetY, offsetZ

  long floatMagX, floatMagY, floatMagZ
  long smoothAccX, smoothAccY, smoothAccZ
  long accToFilterX, accToFilterY, accToFilterZ

  long xMag, yMag, zMag

  long i, recipNorm

  long twoKp, twoKi
  long integralFBx, integralFBy, integralFBz

  long q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3
  long hx, hy, bx, bz
  long halfvx, halfvy, halfvz, halfwx, halfwy, halfwz
  long halfex, halfey, halfez
  long qa, qb, qc
  
OBJ

  IMU         : "MinIMUv2-pasm"                '
  PST         : "Parallax Serial Terminal"
  intmath     : "SL32_INTEngine_2"
    
  fm          : "Float32Full"
  fs          : "FloatString"
    
PUB Init

  IMU.start(SCL,SDA)
  fm.start
  PST.start(115200)
  PST.Clear
   
  Main

PUB Main

  PST.home
  PST.str(string("OpenIMU - Propeller Implementation"))
  PST.newline
  

  IMUinit

  PST.position(35, 1)
  PST.str(string("----Main----"))

  PST.position(35, 16)
  PST.str(string("Pitch: "))
  PST.position(35, 17)
  PST.str(string("Roll: "))
  PST.position(35, 18)
  PST.str(string("Yaw: "))
  
  
  repeat
    rawMagX := fm.ffloat(IMU.getMx)
    rawMagY := fm.ffloat(IMU.getMy)
    rawMagZ := fm.ffloat(IMU.getMz)
   
    rawAcclX := fm.ffloat(IMU.getAx)
    rawAcclY := fm.ffloat(IMU.getAy)
    rawAcclZ := fm.ffloat(IMU.getAz)
   
    rawGyroX := fm.ffloat(IMU.getRx)
    rawGyroY := fm.ffloat(IMU.getRy)
    rawGyroZ := fm.ffloat(IMU.getRz)
    
    PST.position(35,3)
    PST.str(fs.floattostring(rawMagX))
    PST.clearEnd
    PST.position(35,4)
    PST.str(fs.floattostring(rawMagY))
    PST.clearEnd
    PST.position(35,5)
    PST.str(fs.floattostring(rawMagZ))
    PST.clearEnd    
     
    PST.position(35,6)
    PST.str(fs.floattostring(rawAcclX))
    PST.clearEnd
    PST.position(35,7)
    PST.str(fs.floattostring(rawAcclY))
    PST.clearEnd
    PST.position(35,8)
    PST.str(fs.floattostring(rawAcclZ))
    PST.clearEnd
     
    PST.position(35,9)
    PST.str(fs.floattostring(rawGyroX))
    PST.clearEnd
    PST.position(35,10)
    PST.str(fs.floattostring(rawGyroY))
    PST.clearEnd
    PST.position(35,11)
    PST.str(fs.floattostring(rawGyroZ))
    PST.clearEnd
         
    floatMagX := fm.fsub(fm.fmul(fm.fsub(rawMagX, COMPASS_XMIN), INVERSE_XRANGE), 1.0)
    floatMagY := fm.fsub(fm.fmul(fm.fsub(rawMagY, COMPASS_YMIN), INVERSE_YRANGE), 1.0)
    floatMagZ := fm.fsub(fm.fmul(fm.fsub(rawMagZ, COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)
    
    smoothAccX := smoothing(rawAcclX, smoothAccX)
    smoothAccY := smoothing(rawAcclY, smoothAccY)
    smoothAccZ := smoothing(rawAcclZ, smoothAccZ)
   
    PST.position(35,12)
    PST.str(fs.floattostring(smoothAccX))
    PST.clearEnd
    PST.position(35,13)
    PST.str(fs.floattostring(smoothAccY))
    PST.clearEnd
    PST.position(35,14)
    PST.str(fs.floattostring(smoothAccZ))
    PST.clearEnd

    accToFilterX := smoothAccX
    accToFilterY := smoothAccY
    accToFilterZ := smoothAccZ
   
    AHRSupdate

    GetEuler

    UpdateDisplay

PRI UpdateDisplay | pitch2, roll2

  PST.position(42,16)
  PST.str(fs.floattostring(pitch))
  PST.clearEnd
  PST.position(41,17)
  PST.str(fs.floattostring(roll))
  PST.clearEnd
  PST.position(40,18)
  PST.str(fs.floattostring(yaw))
  PST.clearEnd

  PST.position(0,48)
  PST.str(fs.floattostring(q0))
  PST.position(15,48)
  PST.str(fs.floattostring(q1))
  PST.position(30,48)
  PST.str(fs.floattostring(q2))
  PST.position(45,48)
  PST.str(fs.floattostring(q3))
  PST.clearEnd

  pitch2 := fm.Degrees(atan2(rawAcclX, fm.fsqr(fm.fadd(fm.fmul(rawAcclY, rawAcclY), fm.fmul(rawAcclZ, rawAcclZ)))))
  PST.position(7,30)
  pst.str(fs.floattostring(pitch2))
  PST.clearEnd
  
  roll2 := fm.Degrees(atan2(fm.fmul(-1.0, rawAcclY), fm.fsqr(fm.fadd(fm.fmul(rawAcclX, rawAcclX), fm.fmul(rawAcclZ, rawAcclZ)))))
  PST.position(6,31)
  pst.str(fs.floattostring(roll2))   
  PST.clearEnd
  

    
PRI IMUinit

  PST.str(string("----IMUInit----"))
  PST.newline
  PST.str(string("->Calibrating Iteration: "))
  PST.newline
  PST.str(string("rawMagX: "))
  PST.newline
  PST.str(string("rawMagY: "))
  PST.newline
  PST.str(string("rawMagZ: "))
  PST.newline
  
  PST.str(string("rawAcclX: "))
  PST.newline
  PST.str(string("rawAcclY: "))
  PST.newline
  PST.str(string("rawAcclZ: "))
  PST.newline
  
  PST.str(string("rawGyroX: "))
  PST.newline
  PST.str(string("rawGyroY: "))
  PST.newline
  PST.str(string("rawGyroZ: "))
  PST.newline

  PST.str(string("smoothAccX: "))
  PST.newline
  PST.str(string("smoothAccY: "))
  PST.newline
  PST.str(string("smoothAccZ: "))  
  
  repeat i from 0 to 500
    PST.position(25,2)
    PST.dec(i)
    PST.clearEnd  
    
    rawMagX := fm.ffloat(IMU.getMx)
    rawMagY := fm.ffloat(IMU.getMy)
    rawMagZ := fm.ffloat(IMU.getMz)

    PST.position(9,3)
    PST.str(fs.floattostring(rawMagX))
    PST.clearEnd
    PST.position(9,4)
    PST.str(fs.floattostring(rawMagY))
    PST.clearEnd
    PST.position(9,5)
    PST.str(fs.floattostring(rawMagZ))
    PST.clearEnd    
   
    rawAcclX := fm.ffloat(IMU.getAx)
    rawAcclY := fm.ffloat(IMU.getAy)
    rawAcclZ := fm.ffloat(IMU.getAz)

    PST.position(10,6)
    PST.str(fs.floattostring(rawAcclX))
    PST.clearEnd
    PST.position(10,7)
    PST.str(fs.floattostring(rawAcclY))
    PST.clearEnd
    PST.position(10,8)
    PST.str(fs.floattostring(rawAcclZ))
    PST.clearEnd
   
    rawGyroX := fm.ffloat(IMU.getRx)
    rawGyroY := fm.ffloat(IMU.getRy)
    rawGyroZ := fm.ffloat(IMU.getRz)

    PST.position(10,9)
    PST.str(fs.floattostring(rawGyroX))
    PST.clearEnd
    PST.position(10,10)
    PST.str(fs.floattostring(rawGyroY))
    PST.clearEnd
    PST.position(10,11)
    PST.str(fs.floattostring(rawGyroZ))
    PST.clearEnd
    
    smoothAccX := smoothing(rawAcclX, smoothAccX)
    smoothAccY := smoothing(rawAcclY, smoothAccY)
    smoothAccZ := smoothing(rawAcclZ, smoothAccZ)

    PST.position(12,12)
    PST.str(fs.floattostring(smoothAccX))
    PST.clearEnd
    PST.position(12,13)
    PST.str(fs.floattostring(smoothAccY))
    PST.clearEnd
    PST.position(12,14)
    PST.str(fs.floattostring(smoothAccZ))
    PST.clearEnd
    
    waitcnt(MILLI * 3 + cnt)

  PST.position(25,2)
  PST.str(string("Done"))
  PST.clearEnd
  
  PST.position(0,16)
  PST.str(string("Gyro Summing Iteration: "))
  PST.newLine
  
  gyroSumX := gyroSumY := gyroSumZ := 0

  PST.str(string("smoothAccX: "))
  PST.newLine
  PST.str(string("smoothAccY: "))
  PST.newLine
  PST.str(string("smoothAccZ: "))
  PST.newLine
  PST.str(string("gyroSumX: "))
  PST.newLine
  PST.str(string("gyroSumY: "))
  PST.newLine
  PST.str(string("gyroSumZ: "))
  
  repeat i from 0 to 500
    PST.position(24,16)
    PST.dec(i)
    PST.clearEnd
    
    smoothAccX := smoothing(rawAcclX, smoothAccX)
    smoothAccY := smoothing(rawAcclY, smoothAccY)
    smoothAccZ := smoothing(rawAcclZ, smoothAccZ)

    PST.position(12,17)
    PST.str(fs.floattostring(smoothAccX))
    PST.clearEnd
    PST.position(12,18)
    PST.str(fs.floattostring(smoothAccY))
    PST.clearEnd
    PST.position(12,19)
    PST.str(fs.floattostring(smoothAccZ))
    PST.clearEnd          

    gyroSumX += IMU.getRx
    gyroSumY += IMU.getRy
    gyroSumZ += IMU.getRz

    PST.position(10,20)
    PST.dec(gyroSumX)
    PST.clearEnd
    PST.position(10,21)
    PST.dec(gyroSumY)
    PST.clearEnd
    PST.position(10,22)
    PST.dec(gyroSumZ)
    PST.clearEnd          

    waitcnt(MILLI * 3 + cnt)

  PST.position(24,16)
  PST.str(string("Done"))
  PST.clearEnd

  PST.position(0,24)
  PST.str(string("Finding gyro offset: "))
  PST.newLine
  PST.str(string("offSetX: "))
  PST.newLine
  PST.str(string("offSetY: "))
  PST.newLine
  PST.str(string("offSetZ: "))
  
  offSetX := fm.fdiv(fm.ffloat(gyroSumX), 500.0)
  offSetY := fm.fdiv(fm.ffloat(gyroSumY), 500.0)
  offSetZ := fm.fdiv(fm.ffloat(gyroSumZ), 500.0)

  pst.position(9,25)
  PST.str(fs.floattostring(offSetX))
  PST.clearEnd
  pst.position(9,26)
  PST.str(fs.floattostring(offSetY))
  PST.clearEnd
  pst.position(9,27)
  PST.str(fs.floattostring(offSetZ))
  PST.clearEnd

  PST.position(21,24)
  PST.str(string("Done"))

  PST.position(0,29)
  PST.str(string("Finding pitch and roll: "))
  PST.newLine
  PST.str(string("pitch: "))
  PST.newLine
  PST.str(string("roll: "))
  
  PST.position(29,29)
  PST.str(string("pitch"))
  PST.clearEnd
           
  pitch := fm.Degrees(atan2(rawAcclX, fm.fsqr(fm.fadd(fm.fmul(rawAcclY, rawAcclY), fm.fmul(rawAcclZ, rawAcclZ)))))
  PST.position(7,30)
  pst.str(fs.floattostring(pitch))
  PST.clearEnd
  
  PST.position(29,29)
  PST.str(string("roll"))
  PST.clearEnd
  
  roll := fm.Degrees(atan2(fm.fmul(-1.0, rawAcclY), fm.fsqr(fm.fadd(fm.fmul(rawAcclX, rawAcclX), fm.fmul(rawAcclZ, rawAcclZ)))))
  PST.position(6,31)
  pst.str(fs.floattostring(roll))   
  PST.clearEnd
  
  PST.position(29,29)
  PST.str(string("correcting pitch and roll"))
  PST.clearEnd
  
  if rawAcclZ > 0
    if rawAcclX > 0
      pitch := fm.fsub(180.0, pitch)
    else
      pitch := fm.fsub(-180.0, pitch)

    if rawAcclY >0
      roll := fm.fsub(-180.0, roll)
    else
      roll := fm.fsub(180.0, roll)

  PST.position(7,30)
  pst.str(fs.floattostring(pitch))
  PST.clearEnd
  
  PST.position(6,31)
  pst.str(fs.floattostring(roll))   
  PST.clearEnd

  PST.position(29,29)
  PST.str(string("Done"))
  PST.clearEnd

  PST.position(0,34)
  PST.str(string("Finding Yaw: "))

  PST.position(13,34)
  PST.str(string("finding floatMag"))
  PST.clearEnd

  pst.newLine
  PST.str(string("floatMagX: ")) 
  PST.newLine
  PST.str(string("floatMagY: "))
  PST.newLine
  PST.str(string("floatMagZ: "))
  PST.newLine
  PST.str(string("xMag: "))
  PST.newLine
  PST.str(string("yMag: "))
  PST.newLine
  PST.str(string("yaw: "))
  PST.newLine
  
  floatMagX := fm.fsub(fm.fmul(fm.fsub(rawMagX, COMPASS_XMIN), INVERSE_XRANGE), 1.0)
  PST.position(11,35)
  pst.str(fs.floattostring(floatMagX))   
  PST.clearEnd
  
  floatMagY := fm.fsub(fm.fmul(fm.fsub(rawMagY, COMPASS_YMIN), INVERSE_YRANGE), 1.0)
  PST.position(11,36)
  pst.str(fs.floattostring(floatMagY))   
  PST.clearEnd

  floatMagZ := fm.fsub(fm.fmul(fm.fsub(rawMagZ, COMPASS_ZMIN), INVERSE_ZRANGE), 1.0)
  PST.position(11,37)
  pst.str(fs.floattostring(floatMagZ))   
  PST.clearEnd
  
  PST.position(13,34)
  PST.str(string("tilt compensating compass"))
  PST.clearEnd
  
  xMag := fm.fadd(fm.fmul(floatMagX, fm.cos(fm.radians(pitch))), fm.fmul(floatMagZ, fm.sin(fm.radians(pitch))))
  PST.position(6,38)
  pst.str(fs.floattostring(xMag))   
  PST.clearEnd

  yMag := fm.fmul(-1.0, fm.fsub(fm.fadd(fm.fmul(fm.fmul(floatMagX, fm.sin(fm.radians(roll))), fm.sin(fm.radians(pitch))), fm.fmul(floatMagY, fm.cos(fm.radians(roll)))), fm.fmul(fm.fmul(floatMagZ, fm.sin(fm.radians(roll))), fm.cos(fm.radians(pitch)))))
  PST.position(6,39)
  pst.str(fs.floattostring(yMag))   
  PST.clearEnd

  PST.position(13,34)
  PST.str(string("finding yaw"))
  PST.clearEnd
  
  yaw := fm.degrees(atan2(yMag, xMag))

  if yaw < 0.0
    yaw := fm.fadd(yaw, 360.0)

  PST.position(5,40)
  PST.str(fs.floattostring(yaw))
  PST.clearEnd

  PST.position(13,34)
  PST.str(string("Done"))
  PST.clearEnd

  PST.position(0,42)
  PST.str(string("Finding rotation matrix: "))
    
  cosPitch := fm.cos(fm.radians(pitch))
  sinPitch := fm.sin(fm.radians(pitch))
  
  cosRoll := fm.cos(fm.radians(roll))
  sinRoll := fm.sin(fm.radians(roll))                   
  
  cosYaw := fm.cos(fm.radians(yaw))
  sinYaw := fm.sin(fm.radians(yaw))

  printDataLabelFP(30, 35, string("cosPitch: "), cosPitch)
  PST.position(30,35)
  PST.str(string("cosPitch: "))
  PST.str(fs.floattostring(cosPitch))
  PST.position(30,36)
  PST.str(string("sinPitch: "))
  PST.str(fs.floattostring(sinPitch))
  PST.position(30,37)
  PST.str(string("cosRoll: "))
  PST.str(fs.floattostring(cosRoll))
  PST.position(30,38)
  PST.str(string("sinRoll: "))
  PST.str(fs.floattostring(sinRoll))
  PST.position(30,39)
  PST.str(string("cosYaw: "))
  PST.str(fs.floattostring(cosYaw))
  PST.position(30,40)
  PST.str(string("sinYaw: "))
  PST.str(fs.floattostring(sinYaw))

  r11 := fm.fmul(cosPitch, cosYaw)
  r21 := fm.fmul(cosPitch, sinYaw)
  r31 := fm.fmul(-1.0, sinPitch)

  r12 := fm.fadd(fm.fmul(-1.0, fm.fmul(cosRoll, sinYaw)), fm.fmul(fm.fmul(sinRoll, sinPitch), cosYaw))
  r22 := fm.fadd(fm.fmul(cosRoll, cosYaw), fm.fmul(fm.fmul(sinRoll, sinPitch), sinYaw))
  r32 := fm.fmul(sinRoll, cosPitch)

  r13 := fm.fadd(fm.fmul(sinRoll, sinYaw), fm.fmul(fm.fmul(cosRoll, sinPitch), cosYaw))
  r23 := fm.fadd(fm.fmul(-1.0, fm.fmul(sinRoll, cosYaw)), fm.fmul(fm.fmul(cosRoll, sinPitch), sinYaw))
  r33 := fm.fmul(cosRoll, cosPitch)

  printDataFP(0, 43, r11)
  printDataFP(15, 43, r12)
  printDataFP(30, 43, r13)

  printDataFP(0, 44, r21)
  printDataFP(15, 44, r22)
  printDataFP(30, 44, r23)

  printDataFP(0, 45, r31)
  printDataFP(15, 45, r32)
  printDataFP(30, 45, r33)
  
  q0 := fm.fmul(0.5, fm.fsqr(fm.fadd(fm.fadd(1.0, r11), fm.fadd(r22, r33))))
  q1 := fm.fdiv(fm.fsub(r32, r23), fm.fmul(4.0, q0))
  q2 := fm.fdiv(fm.fsub(r13, r31), fm.fmul(4.0, q0)) 
  q3 := fm.fdiv(fm.fsub(r21, r12), fm.fmul(4.0, q0))
                                            
  printHeading(0, 47, string("Quaternions"))
  
  printDataFP(0, 48, q0)
  printDataFP(15, 48, q1)
  printDataFP(30, 48, q2)
  printDataFP(45, 48, q3)                 

  repeat until PST.charIn == "D"

PRI printDataLabelFP(row, column, label, data)

  PST.position(row, column)
  PST.str(data)
  PST.str(fs.floattostring(data))
  PST.clearEnd

PRI printHeading(row, column, data)

  PST.position(row, column)
  PST.str(data)
  PST.clearEnd
  
PRI printLabel(row, column, data)

  PST.position(row, column)
  PST.str(data)
  
PRI printDataFP(row, column, data)

  PST.position(row, column)
  PST.str(fs.floattostring(data))
  PST.clearEnd  
  
PRI AHRSupdate | ax, ay, az, gx, gy, gz, mx, my, mz

'ifnot (ax == 0.0) and (ay == 0.0) and (az == 0.0)
  ' Normalise acclerometer measurements
  ax := fm.fmul(-1.0, rawAcclX)
  ay := fm.fmul(-1.0, rawAcclY)
  az := fm.fmul(-1.0, rawAcclZ)

  printDataFP(50, 3, ax)
  printDataFP(50, 4, ay)
  printDataFP(50, 5, az)

  gx := fm.radians(fm.fmul(fm.fsub(rawGyroX, offSetX), GYRO_SCALE))
  gy := fm.radians(fm.fmul(fm.fsub(rawGyroY, offSetY), GYRO_SCALE))
  gz := fm.radians(fm.fmul(fm.fsub(rawGyroZ, offSetZ), GYRO_SCALE))

  printDataFP(50, 7, gx)
  printDataFP(50, 8, gy)
  printDataFP(50, 9, gz)

  mx := floatMagX
  my := floatMagY
  mz := floatMagZ

  printDataFP(50, 11, mx)
  printDataFP(50, 12, my)
  printDataFP(50, 13, mz)

  recipNorm := invSqrt(fm.fadd(fm.fadd(fm.fmul(ax, ax), fm.fmul(ay, ay)), fm.fmul(az, az)))
  ax := fm.fmul(recipNorm, ax)
  ay := fm.fmul(recipNorm, ay)
  az := fm.fmul(recipNorm, az)

  printDataFP(50, 3, ax)
  printDataFP(50, 4, ay)
  printDataFP(50, 5, az)
 
  ' Normalise magentometer measurements
  recipNorm := invSqrt(fm.fadd(fm.fadd(fm.fmul(mx, mx), fm.fmul(my, my)), fm.fmul(mz, mz)))
  mx := fm.fmul(recipNorm, mx) 
  my := fm.fmul(recipNorm, my)
  mz := fm.fmul(recipNorm, mz)

  printDataFP(50, 11, mx)
  printDataFP(50, 12, my)
  printDataFP(50, 13, mz)
   
  q0q0 := fm.fmul(q0, q0) 
  q0q1 := fm.fmul(q0, q1) 
  q0q2 := fm.fmul(q0, q2) 
  q0q3 := fm.fmul(q0, q3) 
  q1q1 := fm.fmul(q1, q1) 
  q1q2 := fm.fmul(q1, q2) 
  q1q3 := fm.fmul(q1, q3) 
  q2q2 := fm.fmul(q2, q2) 
  q3q3 := fm.fmul(q3, q3)                 

  printDataFP(50, 23, q0q0)
  printDataFP(50, 24, q0q1)
  printDataFP(50, 25, q0q2)
  printDataFP(50, 26, q0q3)
  printDataFP(50, 27, q1q1)
  printDataFP(50, 28, q1q2)
  printDataFP(50, 29, q1q3)
  printDataFP(50, 30, q2q2)
  printDataFP(50, 31, q3q3) 
  
  ' Reference direction of Earth's magnetic field
 
  hx := fm.fmul(2.0, fm.fadd(fm.fadd(fm.fmul(mx, fm.fsub(fm.fsub(0.5, q2q2), q3q3)), fm.fmul(my, fm.fsub(q1q2, q0q3))), fm.fmul(mz, fm.fadd(q1q3, q0q2))))
  hy := fm.fmul(2.0, fm.fadd(fm.fadd(fm.fmul(mx, fm.fadd(q1q2, q0q3)), fm.fmul(my, fm.fsub(fm.fsub(0.5, q1q1), q3q3))), fm.fmul(mz, fm.fsub(q2q3, q0q1)))) 
  bx := fm.fsqr(fm.fadd(fm.fmul(hx, hx), fm.fmul(hy, hy)))
  bz := fm.fmul(2.0, fm.fadd(fm.fadd(fm.fmul(mx, fm.fsub(q1q3, q0q2)), fm.fmul(my, fm.fadd(q2q3, q0q1))), fm.fmul(mz, fm.fsub(fm.fsub(0.5, q1q1), q2q2))))
   
  ' Estimated direction of gravity and magnetic field
  halfvx := fm.fsub(q1q3, q0q2)
  halfvy := fm.fadd(q0q1, q2q3)
  halfvz := fm.fadd(fm.fsub(q0q0, 0.5), q3q3)
  halfwx := fm.fadd(fm.fmul(bx, fm.fsub(fm.fsub(0.5, q2q2), q3q3)), fm.fmul(bz, fm.fsub(q1q3, q0q2)))
  halfwy := fm.fadd(fm.fmul(bx, fm.fsub(q1q2, q0q3)), fm.fmul(bz, fm.fadd(q0q1, q2q3)))
  halfwz := fm.fadd(fm.fmul(bx, fm.fadd(q0q2, q1q3)), fm.fmul(bz, fm.fsub(fm.fsub(0.5, q1q1), q2q2)))
   
  ' Error is sum of cross product between estimated direction and measured direction of field vectors
  halfex := fm.fadd(fm.fsub(fm.fmul(ay, halfvz), fm.fmul(az, halfvy)), fm.fsub(fm.fmul(my, halfwz), fm.fmul(mz, halfwy)))
  halfey := fm.fadd(fm.fsub(fm.fmul(az, halfvx), fm.fmul(ax, halfvz)), fm.fsub(fm.fmul(mz, halfwx), fm.fmul(mx, halfwz))) 
  halfez := fm.fadd(fm.fsub(fm.fmul(ax, halfvy), fm.fmul(ay, halfvx)), fm.fsub(fm.fmul(mx, halfwy), fm.fmul(my, halfwx)))
   
  ' Compute and apply integral feedback if enabled
  if twoKi > 0.0
    integralFBx := fm.fadd(integralFBx, fm.fmul(twoKi, fm.fmul(halfex, fm.fdiv(1.0, SAMPLE_FREQ))))
    integralFBy := fm.fadd(integralFBx, fm.fmul(twoKi, fm.fmul(halfey, fm.fdiv(1.0, SAMPLE_FREQ))))
    integralFBz := fm.fadd(integralFBx, fm.fmul(twoKi, fm.fmul(halfez, fm.fdiv(1.0, SAMPLE_FREQ))))
   
  else
    integralFBx := 0.0  
    integralFBy := 0.0
    integralFBz := 0.0
   
  gx := fm.fadd(gx, fm.fmul(twoKp, halfex))
  gy := fm.fadd(gy, fm.fmul(twoKp, halfey))
  gz := fm.fadd(gz, fm.fmul(twoKp, halfez))
  
  gx := fm.fmul(gx, fm.fmul(0.5, fm.fdiv(1.0, SAMPLE_FREQ)))
  gy := fm.fmul(gx, fm.fmul(0.5, fm.fdiv(1.0, SAMPLE_FREQ)))
  gz := fm.fmul(gx, fm.fmul(0.5, fm.fdiv(1.0, SAMPLE_FREQ)))
  qa := q0
  qb := q1
  qc := q2
  q0 := fm.fadd(q0, fm.fsub(fm.fsub(fm.fmul(fm.fneg(qb), gz), fm.fmul(qc, gy)), fm.fmul(q3, gz)))
  q1 := fm.fadd(q1, fm.fsub(fm.fadd(fm.fmul(qa, gx), fm.fmul(qc, gz)), fm.fmul(q3, gy)))
  q2 := fm.fadd(q2, fm.fadd(fm.fsub(fm.fmul(qa, gy), fm.fmul(qb, gz)), fm.fmul(q3, gx)))
  q3 := fm.fadd(q3, fm.fsub(fm.fadd(fm.fmul(qa, gz), fm.fmul(qb, gy)), fm.fmul(qc, gx)))
  
  ' Normalise quaternion
  recipNorm := invSqrt(fm.fadd(fm.fadd(fm.fadd(fm.fmul(q0, q0), fm.fmul(q1, q1)), fm.fmul(q2, q2)), fm.fmul(q3, q3)))
  q0 := fm.fmul(q0, recipNorm)
  q1 := fm.fmul(q1, recipNorm)
  q2 := fm.fmul(q2, recipNorm)
  q3 := fm.fmul(q3, recipNorm)
  
PRI invSqrt(x) 

  return(fm.pow(x, -0.5))

PRI GetEuler

  roll := fm.degrees(atan2(fm.fmul(2.0, fm.fadd(fm.fmul(q0, q1), fm.fmul(q2, q3))), fm.fsub(1.0, fm.fmul(2.0, fm.fadd(fm.fmul(q1, q1), fm.fmul(q2, q2))))))
  pitch := fm.degrees(fm.asin(fm.fmul(2.0, fm.fsub(fm.fmul(q0, q2), fm.fmul(q3, q1)))))
  yaw := fm.degrees(atan2(fm.fmul(2.0, fm.fadd(fm.fmul(q0, q3), fm.fmul(q1, q2))), fm.fsub(1.0, fm.fmul(2.0, fm.fadd(fm.fmul(q2, q2), fm.fmul(q3, q3))))))

  if yaw < 0.0
    yaw := fm.fadd(yaw, 360.0)
    
PRI Smoothing(rawAccl, smoothedAccl)

  return(fm.fadd(fm.fmul(rawAccl, 0.15), fm.fmul(smoothedAccl, 0.85)))

PRI atan2(y, x) | z, atan

  if x == 0.0
    if y > 0.0
      return(PIBY2_FLOAT)
    if y == 0.0
      return(0.0)
    else
      return(fm.fneg(PIBY2_FLOAT))

  z := fm.fdiv(y, x)
  if fm.fabs(z) < 1.0
    atan := fm.fdiv(z, fm.fadd(1.0,fm.fmul(fm.fmul(0.28, z), z)))

    if x < 0.0
      if y < 0.0
        return(fm.fsub(atan, PI_FLOAT))
      else
        return(fm.fadd(atan, PI_FLOAT))

  else
    atan := fm.fsub(PIBY2_FLOAT, fm.fdiv(z , fm.fadd(fm.fmul(z, z), 0.28)))
    if y < 0.0
      return(fm.fsub(atan, PI_FLOAT))

  return(atan)
                                                 